/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>       
#include <GL/glut.h>
#include <stdbool.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

/* macros */
#define IX(i,j) ((i)+(N+2)*(j))

/* external definitions (from solver.c) */
extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, float*, float*, float* );

/* global variables */
static int N;
static float dt, diff, visc;
static float force, source;
static int is_drawing_wall = 0; // Flag para alternar entre desenho de paredes e adição de tinta
static int dvel;
static int type;

// Matrizes de Velocidade
static float * u, * v, * u_prev, * v_prev;

// Matrizes de Densidade RGB
static float *dens_r, *dens_r_prev;
static float *dens_g, *dens_g_prev;
static float *dens_b, *dens_b_prev;

// Matrizes de Bloom RGB
static float *dens_bloom_r, *ping_r, *pong_r;
static float *dens_bloom_g, *ping_g, *pong_g;
static float *dens_bloom_b, *ping_b, *pong_b;

// Matrizes de Colisão
static float *walls, *walls_norm_u, *walls_norm_v;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;
static float current_r = 1.0f, current_g = 1.0f, current_b = 1.0f; // Cor atual do pincel

/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/

static void free_data ( void )
{
    if ( u ) free ( u );
    if ( v ) free ( v );
    if ( u_prev ) free ( u_prev );
    if ( v_prev ) free ( v_prev );
    
    // Libera RGB
    if ( dens_r ) free ( dens_r ); if ( dens_r_prev ) free ( dens_r_prev );
    if ( dens_g ) free ( dens_g ); if ( dens_g_prev ) free ( dens_g_prev );
    if ( dens_b ) free ( dens_b ); if ( dens_b_prev ) free ( dens_b_prev );

    // Libera Bloom RGB
    if ( dens_bloom_r ) free ( dens_bloom_r ); if ( ping_r ) free ( ping_r ); if ( pong_r ) free ( pong_r );
    if ( dens_bloom_g ) free ( dens_bloom_g ); if ( ping_g ) free ( ping_g ); if ( pong_g ) free ( pong_g );
    if ( dens_bloom_b ) free ( dens_bloom_b ); if ( ping_b ) free ( ping_b ); if ( pong_b ) free ( pong_b );
    
    if ( walls ) free ( walls );
    if (walls_norm_u) free(walls_norm_u);
    if (walls_norm_v) free(walls_norm_v);
}

float get_density_from_color(unsigned char r, unsigned char g, unsigned char b) {
    if (r > 200 && g > 200 && b > 200) return 1.0f;
    return 0.0f;
}

void compute_normals() {
    const float SQRT1_2 = 0.7071067811865476f;

    for (int i = 1; i <= N; ++i) {
        for (int j = 1; j <= N; ++j) {
            walls_norm_u[IX(i, j)] += SQRT1_2 * walls[IX(i - 1, j + 1)] + 0.0f * walls[IX(i, j + 1)] - SQRT1_2 * walls[IX(i + 1, j + 1)];
            walls_norm_u[IX(i, j)] += 1.0f    * walls[IX(i - 1, j)]     + 0.0f * walls[IX(i, j)]     - 1.0f    * walls[IX(i + 1, j)];
            walls_norm_u[IX(i, j)] += SQRT1_2 * walls[IX(i - 1, j - 1)] + 0.0f * walls[IX(i, j - 1)] - SQRT1_2 * walls[IX(i + 1, j - 1)];
            
            walls_norm_v[IX(i, j)] += -SQRT1_2 * walls[IX(i - 1, j + 1)] - 1.0f * walls[IX(i, j + 1)] - SQRT1_2 * walls[IX(i + 1, j + 1)];
            walls_norm_v[IX(i, j)] += 0.0f     * walls[IX(i - 1, j)]     + 0.0f * walls[IX(i, j)]     - 0.0f    * walls[IX(i + 1, j)];
            walls_norm_v[IX(i, j)] += SQRT1_2  * walls[IX(i - 1, j - 1)] + 1.0f * walls[IX(i, j - 1)] + SQRT1_2 * walls[IX(i + 1, j - 1)];
            
            float norm = sqrtf(walls_norm_u[IX(i, j)] * walls_norm_u[IX(i, j)] + walls_norm_v[IX(i, j)] * walls_norm_v[IX(i, j)]);
            if (norm < 1e-3) continue;
            walls_norm_u[IX(i, j)] /= norm;
            walls_norm_v[IX(i, j)] /= norm;
        }
    }
}

static void clear_data ( void )
{
    int i, size=(N+2)*(N+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = walls[i] = walls_norm_u[i] = walls_norm_v[i] = 0.0f;
        
        // Zera tudo no RGB
        dens_r[i] = dens_r_prev[i] = dens_bloom_r[i] = ping_r[i] = pong_r[i] = 0.0f;
        dens_g[i] = dens_g_prev[i] = dens_bloom_g[i] = ping_g[i] = pong_g[i] = 0.0f;
        dens_b[i] = dens_b_prev[i] = dens_bloom_b[i] = ping_b[i] = pong_b[i] = 0.0f;
    }

    // draw image into density map
    int w, h, n;
    unsigned char *img = stbi_load("logo_matind.jpg", &w, &h, &n, 3);
    if (img == NULL) {
        printf("Failure reason: %s\n", stbi_failure_reason());
        exit(1);
    }

    int max_size = N + 2;
    for (int i = 1; i < max_size-1; ++i) {
        for (int j = 1; j < max_size-1; ++j) {
            int img_i = (i * h) / max_size;
            int img_j = (j * w) / max_size;
            int img_idx = (img_i * w + img_j) * 3;

            if (type){
                dens_r[IX(j, max_size - i - 1)] = img[img_idx + 0] / 255.0f;
                dens_g[IX(j, max_size - i - 1)] = img[img_idx + 1] / 255.0f;
                dens_b[IX(j, max_size - i - 1)] = img[img_idx + 2] / 255.0f;
            } else {
                walls[IX(j, max_size - i - 1)] = get_density_from_color(img[img_idx + 0], img[img_idx + 1], img[img_idx + 2]);
            }
        }
    }

    stbi_image_free(img);

    compute_normals();
}

static int allocate_data ( void )
{
    int size = (N+2)*(N+2);

    u           = (float *) malloc ( size*sizeof(float) );
    v           = (float *) malloc ( size*sizeof(float) );
    u_prev      = (float *) malloc ( size*sizeof(float) );
    v_prev      = (float *) malloc ( size*sizeof(float) );
    
    // Aloca RGB
    dens_r      = (float *) malloc ( size*sizeof(float) );
    dens_r_prev = (float *) malloc ( size*sizeof(float) );
    dens_g      = (float *) malloc ( size*sizeof(float) );
    dens_g_prev = (float *) malloc ( size*sizeof(float) );
    dens_b      = (float *) malloc ( size*sizeof(float) );
    dens_b_prev = (float *) malloc ( size*sizeof(float) );

    // Aloca Bloom RGB
    dens_bloom_r = (float *) malloc ( size*sizeof(float) );
    ping_r       = (float *) malloc ( size*sizeof(float) );
    pong_r       = (float *) malloc ( size*sizeof(float) );
    
    dens_bloom_g = (float *) malloc ( size*sizeof(float) );
    ping_g       = (float *) malloc ( size*sizeof(float) );
    pong_g       = (float *) malloc ( size*sizeof(float) );
    
    dens_bloom_b = (float *) malloc ( size*sizeof(float) );
    ping_b       = (float *) malloc ( size*sizeof(float) );
    pong_b       = (float *) malloc ( size*sizeof(float) );
    
    walls        = (float *) malloc ( size*sizeof(float) );
    walls_norm_u = (float*) malloc ( size * sizeof(float) );
    walls_norm_v = (float*) malloc ( size * sizeof(float) );

    if ( !u || !dens_r || !dens_bloom_r || !walls ) {
        fprintf ( stderr, "cannot allocate data\n" );
        return ( 0 );
    }

    return ( 1 );
}

/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
    glViewport ( 0, 0, win_x, win_y );
    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity ();
    gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
    glutSwapBuffers ();
}

static void draw_velocity ( void )
{
    int i, j;
    float x, y, h;

    h = 1.0f/N;

    glColor3f ( 1.0f, 1.0f, 1.0f );
    glLineWidth ( 1.0f );

    glBegin ( GL_LINES );

        for ( i=1 ; i<=N ; i++ ) {
            x = (i-0.5f)*h;
            for ( j=1 ; j<=N ; j++ ) {
                y = (j-0.5f)*h;
                glVertex2f ( x, y );
                glVertex2f ( x+walls_norm_u[IX(i,j)] * 0.03f, y+walls_norm_v[IX(i,j)] * 0.03f );
            }
        }

    glEnd ();
}

int clamp(int v, int a, int b) {
    return v < a ? a : (v > b ? b : v);
}

void blur_pass(float* src, float* dest, bool horizontal) {
    float weight[] = { 0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216 };
    for (int i = 1; i <= N; ++i) {
        for (int j = 1; j <= N; ++j) {
            float result = src[IX(i, j)] * weight[0];
            float mult1 = 1.0f, mult2 = 1.0f;
            for (int k = 1; k < 5; ++k) {
                if (horizontal) {
                    if (walls[IX(clamp(i + k, 1, N), j)] == 1.0f) mult1 = 0.0f;
                    if (walls[IX(clamp(i - k, 1, N), j)] == 1.0f) mult2 = 0.0f;
                    result += src[IX(clamp(i + k, 1, N), j)] * weight[k] * mult1;
                    result += src[IX(clamp(i - k, 1, N), j)] * weight[k] * mult2;
                } else {
                    if (walls[IX(i, clamp(j + k, 1, N))] == 1.0f) mult1 = 0.0f;
                    if (walls[IX(i, clamp(j - k, 1, N))] == 1.0f) mult2 = 0.0f;
                    result += src[IX(i, clamp(j + k, 1, N))] * weight[k] * mult1;
                    result += src[IX(i, clamp(j - k, 1, N))] * weight[k] * mult2;
                }
            }
            dest[IX(i, j)] = result;
        }
    }
}

void bloom_channel(float* src, float* p_ping, float* p_pong, float* dest, float threshold, int passes, float exposure, float gamma) {
    for (int i = 1; i <= N; ++i) {
        for (int j = 1; j <= N; ++j) {
            p_ping[IX(i, j)] = src[IX(i, j)] > threshold ? src[IX(i, j)] : 0.0f;
        }
    }
    for (int pass = 0; pass < passes; ++pass) {
        bool horizontal = (pass & 1) == 0;
        blur_pass(horizontal ? p_ping : p_pong, horizontal ? p_pong : p_ping, horizontal);
    }
    for (int i = 1; i <= N; ++i) {
        for (int j = 1; j <= N; ++j) {
            if ((passes & 1) == 0) {
                dest[IX(i, j)] = src[IX(i, j)] + p_ping[IX(i, j)];
            } else {
                dest[IX(i, j)] = src[IX(i, j)] + p_pong[IX(i, j)];
            }
            dest[IX(i, j)] = powf(1.0f - expf(-dest[IX(i, j)] * exposure), 1.0f / gamma);
        }
    }
}

static void draw_density ( void )
{
    // Calcula o Bloom separadamente para R, G e B
    bloom_channel(dens_r, ping_r, pong_r, dens_bloom_r, 0.9f, 10, 1.2f, 1.3f);
    bloom_channel(dens_g, ping_g, pong_g, dens_bloom_g, 0.9f, 10, 1.2f, 1.3f);
    bloom_channel(dens_b, ping_b, pong_b, dens_bloom_b, 0.9f, 10, 1.2f, 1.3f);

    int i, j;
    float x, y, h;

    h = 1.0f/N;

    glBegin ( GL_QUADS );

    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            int idx00 = IX(i, j);
            int idx10 = IX(i+1, j);
            int idx11 = IX(i+1, j+1);
            int idx01 = IX(i, j+1);

            // Vértice 00 (Renderizando paredes em branco)
            if (walls[idx00] > 0.0f) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(dens_bloom_r[idx00], dens_bloom_g[idx00], dens_bloom_b[idx00]);
            glVertex2f ( x, y );

            // Vértice 10
            if (walls[idx10] > 0.0f) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(dens_bloom_r[idx10], dens_bloom_g[idx10], dens_bloom_b[idx10]);
            glVertex2f ( x+h, y );

            // Vértice 11
            if (walls[idx11] > 0.0f) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(dens_bloom_r[idx11], dens_bloom_g[idx11], dens_bloom_b[idx11]);
            glVertex2f ( x+h, y+h );

            // Vértice 01
            if (walls[idx01] > 0.0f) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(dens_bloom_r[idx01], dens_bloom_g[idx01], dens_bloom_b[idx01]);
            glVertex2f ( x, y+h );
        }
    }

    glEnd ();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI ( float * dr, float * dg, float * db, float * u_ui, float * v_ui )
{
    int i, j, size = (N+2)*(N+2);

    for ( i=0 ; i<size ; i++ ) {
        u_ui[i] = v_ui[i] = dr[i] = dg[i] = db[i] = 0.0f;
    }

    if ( !mouse_down[0] && !mouse_down[2] ) return;

    i = (int)((       mx /(float)win_x)*N+1);
    j = (int)(((win_y-my)/(float)win_y)*N+1);

    if ( i<1 || i>N || j<1 || j>N ) return;

    // Splat (raio) do mouse
    int radius = 2; 
    
    for (int ix = -radius; ix <= radius; ix++) {
        for (int iy = -radius; iy <= radius; iy++) {
            int cx = clamp(i + ix, 1, N);
            int cy = clamp(j + iy, 1, N);
            int index = IX(cx, cy);

            // Botão Esquerdo: Vento
            if ( mouse_down[0] ) {
                u_ui[index] = force * (mx-omx);
                v_ui[index] = force * (omy-my);
            }

            // Botão Direito: Tinta com cor aleatória
            if ( mouse_down[2] ) {
                if (is_drawing_wall) {
                    walls[index] = 1.0f; // Marca como parede
                } else {
                    dr[index] = current_r * source;
                    dg[index] = current_g * source;
                    db[index] = current_b * source;
                }
            }
        }
    }

    compute_normals();

    omx = mx;
    omy = my;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
    switch ( key )
    {
        case 'c':
        case 'C':
            clear_data ();
            break;

        case 'q':
        case 'Q':
            free_data ();
            exit ( 0 );
            break;

        case 'v':
        case 'V':
            dvel = !dvel;
            break;

        case 'w':
        case 'W':
            is_drawing_wall = !is_drawing_wall;
            break;
    }

}

static void mouse_func ( int button, int state, int x, int y )
{
    omx = mx = x;
    omy = my = y; // Corrigido

    mouse_down[button] = state == GLUT_DOWN;

    // Gera cor nova ao clicar com direito
    if (button == 2 && state == GLUT_DOWN) {

        current_r = (float)rand() / RAND_MAX;
        current_g = (float)rand() / RAND_MAX;
        current_b = (float)rand() / RAND_MAX;
    }
}

static void motion_func ( int x, int y )
{
    mx = x;
    my = y;
}

static void reshape_func ( int width, int height )
{
    glutSetWindow ( win_id );
    glutReshapeWindow ( width, height );

    win_x = width;
    win_y = height;
}

static void idle_func ( void )
{
    // Usa UI no lugar das gotas automáticas
    get_from_UI ( dens_r_prev, dens_g_prev, dens_b_prev, u_prev, v_prev );

    vel_step ( N, u, v, u_prev, v_prev, visc, dt, walls_norm_u, walls_norm_v, walls );
    
    #pragma omp parallel sections
    {
        #pragma omp section
        dens_step ( N, dens_r, dens_r_prev, u, v, diff, dt );
        
        #pragma omp section
        dens_step ( N, dens_g, dens_g_prev, u, v, diff, dt );
        
        #pragma omp section
        dens_step ( N, dens_b, dens_b_prev, u, v, diff, dt ); 
    }
    
    glutPostRedisplay ();
}

static void display_func ( void )
{
    pre_display ();

        if ( dvel ) draw_velocity ();
        else        draw_density ();

    post_display ();
}


/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( win_x, win_y );
    win_id = glutCreateWindow ( "Fluido Interativo - Universo UFPR 2026" );

    glClearColor ( 1.0f, 1.0f, 1.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();

    pre_display ();

    glutKeyboardFunc ( key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutIdleFunc ( idle_func );
    glutDisplayFunc ( display_func );
}


/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
    glutInit ( &argc, argv );

    if ( argc > 8 ) {
        fprintf ( stderr, "usage : %s type N dt diff visc force source\n", argv[0] );
        fprintf ( stderr, "where:\n" );\
        fprintf ( stderr, "\t type   : simulation type (0 for obstacles and 1 for no obstacles)\n" );
        fprintf ( stderr, "\t N      : grid resolution\n" );
        fprintf ( stderr, "\t dt     : time step\n" );
        fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
        fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
        fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
        fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
        exit ( 1 );
    }

    type = 0;
    N = 256;
    dt = 0.1f;
    diff = 0.0f;
    visc = 0.0f;
    force = 5.0f;
    source = 100.0f;

    switch (argc) {
        case 8:
            source = atof(argv[7]);
        case 7:
            force = atof(argv[6]);
        case 6:
            visc = atof(argv[5]);
        case 5:
            diff = atof(argv[4]);
        case 4:
            dt = atof(argv[3]);
        case 3:
            N = atoi(argv[2]);
        case 2:
            type = atoi(argv[1]);
            break;
    }

    if ( argc == 1 ) {
        fprintf ( stderr, "Using defaults : type=%d N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
            type, N, dt, diff, visc, force, source );
    }

    printf ( "\n\nComo usar esta demo:\n\n" );
    printf ( "\t Adicione densidades com o botão direito do mouse (Cores aleatorias!)\n" );
    printf ( "\t Adicione velocidades com o botão esquerdo do mouse e arrastando o mouse\n" );
    printf ( "\t Alternar a exibição de densidade/velocidade com tecla 'v'\n" );
    printf ( "\t Limpar a simulação com a tecla 'c'\n" );
    printf ( "\t Para sair pressione a tecla 'q'\n" );

    dvel = 0;

    if ( !allocate_data () ) exit ( 1 );
    clear_data ();


    win_x = 512;
    win_y = 512;
    open_glut_window ();

    glutMainLoop ();

    exit ( 0 );
}