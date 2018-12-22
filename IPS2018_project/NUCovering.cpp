#pragma once
#include "fragmentation.h"
#include <locale.h>
#include <chrono>
#include <cilk\cilk.h>
#include <cilk\reducer_vector.h>
#include <cilk\cilk_api.h>

/// параметры начальной прямоугольной области
const double g_l1_max = 12.0;
const double g_l2_max = g_l1_max;
const double g_l1_min = 8.0;
const double g_l2_min = g_l1_min;
const double g_l0 = 5.0;

/// точность аппроксимации рабочего пространства
const double g_precision = 0.25;

int main()
{
	setlocale(LC_ALL,"Rus");

	__cilkrts_end_cilk();
	__cilkrts_set_param("nworkers", "6");
	
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	high_level_analysis main_object;
	main_object.GetSolution();

	end = std::chrono::system_clock::now();
	int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>
		(end - start).count();
	// timer 
	//printf("Number of workers %i\n", __cilkrts_get_nworkers());
	printf("Elapsed time %i ms.\n", elapsed_ms);
	const char* out_files = "out.txt";
	//WriteResults(out_files);
	getchar();

	return 0;
}