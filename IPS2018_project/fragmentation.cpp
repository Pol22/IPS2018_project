#include "fragmentation.h"

/// вектор, содержащий box-ы, €вл€ющиес€ частью рабочего пространства
std::vector<Box> solution;
/// вектор, содержащий box-ы, не €вл€ющиес€ частью рабочего пространства
std::vector<Box> not_solution;
/// вектор, содержащий box-ы, наход€щиес€ на границе между "рабочим" и "нерабочим" пространством
std::vector<Box> boundary;
/// вектор, хран€щий box-ы, анализируемые на следующей итерации алгоритма
std::vector<Box> temporary_boxes;

/// функции gj()
//------------------------------------------------------------------------------------------
double g1(double x1, double x2)
{
	return (x1*x1 + x2*x2 - g_l1_max*g_l1_max);
}

//------------------------------------------------------------------------------------------
double g2(double x1, double x2)
{
	return (g_l1_min*g_l1_min - x1*x1 - x2*x2);
}

//------------------------------------------------------------------------------------------
double g3(double x1, double x2)
{
	return (x1*x1 + x2*x2 - g_l2_max*g_l2_max);
}

//------------------------------------------------------------------------------------------
double g4(double x1, double x2)
{
	return (g_l2_min*g_l2_min - x1*x1 - x2*x2);
}


//------------------------------------------------------------------------------------------
low_level_fragmentation::low_level_fragmentation(double& min_x, double& min_y, double& x_width, double& y_height )
{
	current_box = Box( min_x, min_y, x_width, y_height );
}

//------------------------------------------------------------------------------------------
low_level_fragmentation::low_level_fragmentation(const Box& box)
{
	current_box = box;
}

//------------------------------------------------------------------------------------------
void low_level_fragmentation::VerticalSplitter(const Box& box, boxes_pair& vertical_splitter_pair)
{
	double x, y, width, height;
	box.GetParameters(x, y, width, height);
	vertical_splitter_pair.first = Box(x, y, width, height / 2);
	vertical_splitter_pair.second = Box(x, y + height / 2, width, height / 2);
}

//------------------------------------------------------------------------------------------
void low_level_fragmentation::HorizontalSplitter(const Box& box, boxes_pair& horizontal_splitter_pair)
{
	double x, y, width, height;
	box.GetParameters(x, y, width, height);
	horizontal_splitter_pair.first = Box(x, y, width / 2, height);
	horizontal_splitter_pair.second = Box(x + width / 2, y, width / 2, height);
}

//------------------------------------------------------------------------------------------
void low_level_fragmentation::GetNewBoxes(const Box& box, boxes_pair& new_pair_of_boxes)
{
	double x, y, width, height;
	box.GetParameters(x, y, width, height);
	if (width > height)
		HorizontalSplitter(box, new_pair_of_boxes);
	else
		VerticalSplitter(box, new_pair_of_boxes);
}

//------------------------------------------------------------------------------------------
unsigned int low_level_fragmentation::FindTreeDepth()
{
	double box_diagonal = current_box.GetDiagonal();

	if (box_diagonal <= g_precision)
	{
		return 0;
	}
	else
	{
		boxes_pair new_boxes;
		// допустим, разобьем начальную область по ширине
		VerticalSplitter(current_box, new_boxes);
		unsigned int tree_depth = 1;

		box_diagonal = new_boxes.first.GetDiagonal();

		if (box_diagonal <= g_precision)
		{
			return tree_depth;
		}
		else
		{
			for (;;)
			{
				GetNewBoxes(new_boxes.first, new_boxes);
				++tree_depth;
				box_diagonal = new_boxes.first.GetDiagonal();

				if (box_diagonal <= g_precision)
				{
					break;
				}
			}
			return tree_depth;
		}
	}
}

//------------------------------------------------------------------------------------------
int low_level_fragmentation::ClasifyBox(const min_max_vectors& vects)
{
	return 0;
}

//------------------------------------------------------------------------------------------
void low_level_fragmentation::GetBoxType(const Box& box)
{
	// необходимо определить функцию
}


//------------------------------------------------------------------------------------------
high_level_analysis::high_level_analysis( double& min_x, double& min_y, double& x_width, double& y_height ) :
					low_level_fragmentation(min_x, min_y, x_width, y_height) {}

//------------------------------------------------------------------------------------------
high_level_analysis::high_level_analysis( Box& box ) : low_level_fragmentation( box ) {}

//------------------------------------------------------------------------------------------
void high_level_analysis::GetMinMax( const Box& box, min_max_vectors& min_max_vecs )
{
	std::vector<double> g_min;
	std::vector<double> g_max;

	double a1min, a2min, a1max, a2max;
	double xmin, xmax, ymin, ymax;

	box.GetParameters(xmin, ymin, xmax, ymax);

	xmax = xmin + xmax;
	ymax = ymin + ymax;

	double curr_box_diagonal = box.GetDiagonal();

	if (curr_box_diagonal <= g_precision)
	{
		g_min.push_back(0);
		g_max.push_back(0);

		min_max_vecs.first = g_min;
		min_max_vecs.second = g_max;

		return;
	}

	// MIN
	// функци€ g1(x1,x2)
	a1min = __min(abs(xmin), abs(xmax));
	a2min = __min(abs(ymin), abs(ymax));
	g_min.push_back(g1(a1min, a2min));

	// функци€ g2(x1,x2)
	a1min = __max(abs(xmin), abs(xmax));
	a2min = __max(abs(ymin), abs(ymax));
	g_min.push_back(g2(a1min, a2min));

	// функци€ g3(x1,x2)
	a1min = __min(abs(xmin - g_l0), abs(xmax - g_l0));
	a2min = __min(abs(ymin), abs(ymax));
	g_min.push_back(g3(a1min, a2min));

	// функци€ g4(x1,x2)
	a1min = __max(abs(xmin - g_l0), abs(xmax - g_l0));
	a2min = __max(abs(ymin), abs(ymax));
	g_min.push_back(g4(a1min, a2min));

	// MAX
	// функци€ g1(x1,x2)
	a1max = __max(abs(xmin), abs(xmax));
	a2max = __max(abs(ymin), abs(ymax));
	g_max.push_back(g1(a1max, a2max));

	// функци€ g2(x1,x2)
	a1max = __min(abs(xmin), abs(xmax));
	a2max = __min(abs(ymin), abs(ymax));
	g_max.push_back(g2(a1max, a2max));

	// функци€ g3(x1,x2)
	a1max = __max(abs(xmin - g_l0), abs(xmax - g_l0));
	a2max = __max(abs(ymin), abs(ymax));
	g_max.push_back(g3(a1max, a2max));

	// функци€ g4(x1,x2)
	a1max = __min(abs(xmin - g_l0), abs(xmax - g_l0));
	a2max = __min(abs(ymin), abs(ymax));
	g_max.push_back(g4(a1max, a2max));

	min_max_vecs.first = g_min;
	min_max_vecs.second = g_max;
}

//------------------------------------------------------------------------------------------
void high_level_analysis::GetSolution()
{
	double left = -g_l1_max;
	double top = 0;
	double width = g_l1_max + g_l0 + g_l2_max;
	double height = std::min(g_l1_max, g_l2_max);
	Box init = Box(left, top, width, height);
	temporary_boxes.push_back(init);

	double max_diam = init.GetDiagonal();
	printf("Init diameter of the initial rect is %f\n", max_diam);

	bool bExit(false);
	int curLevelNodes(0);
	Box* current_box;
	double current_diam(0);
	boxes_pair pair;

	for (int curLevel = 0; curLevel < maxLevels; curLevel++)
	{
		//printf("Processing level %i\n", curLevel);

		curLevelNodes = temporary_boxes.size();
		cilk::reducer<cilk::op_vector<Box>> r;

		for (int curLevelNode = 0; curLevelNode < curLevelNodes; curLevelNode++)
		{
			current_box = &temporary_boxes[curLevelNode];

			current_diam = current_box->GetDiagonal();
			if (current_diam < max_diam)
			{
				max_diam = current_diam;
				//printf("Current level diameter of the rectangle is %f\n", max_diam);
			}

			bool inQE(false);
			bool inQI(false);

			if (current_diam <= g_precision)
			{
				solution.push_back(*current_box);
				// exit
				bExit = true;
			}
			else
			{
				min_max_vectors vec;
				GetMinMax(*current_box, vec);
				
				GetNewBoxes(*current_box, pair);
				r->push_back(pair.first);
				r->push_back(pair.second);
			}
		}
		temporary_boxes.clear();
		r.move_out(temporary_boxes);

		if (bExit)
		{
			//printf("The result is obtained for %i levels\n", curLevel);
			break;
		}

	}

}


//------------------------------------------------------------------------------------------
void WriteResults(const char* file_name)
{
	FILE* fp;
	if ((fp = fopen(file_name, "wb")) == NULL) 
	{
		printf("Cannot open file %s.\n", file_name);
		exit(1);
	}
	char enter = '\n';
	for (int i = 0; i < solution.size(); i++)
	{
		double x, y, width, height;
		solution[i].GetParameters(x, y, width, height);
		fwrite(&x, sizeof(double), 1, fp);
		fwrite(&y, sizeof(double), 1, fp);
		fwrite(&width, sizeof(double), 1, fp);
		fwrite(&height, sizeof(double), 1, fp);
		fwrite(&enter, sizeof(char), 1, fp);
	}
	fclose(fp);
}