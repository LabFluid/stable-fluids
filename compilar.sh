GCC_COMMAND="gcc solver.c demo.c -fopenmp -lGL -lGLU -lglut -O3 -lm -march=native -flto -lm -Wall "

if [ -z "$1" ]; then
	$GCC_COMMAND -o demo -DFIN &
	$GCC_COMMAND -o demo_mag -DFIN -DMAG
	wait
elif [ "$1" = "inf" ]; then
	$GCC_COMMAND -o demo  &
	$GCC_COMMAND -o demo_mag -DMAG
	wait
elif [ "$1" = "r" ]; then
	./demo &
	./demo_mag
	wait
	gnuplot -p -e 'set xlabel "Tempo (num. de passos)"; set ylabel "Energia Cinética"; set logscale y; plot "energy.txt" with linespoints title "Energia", "energy_mag.txt" with linespoints title "Energia com magnitude preservada"'
fi
