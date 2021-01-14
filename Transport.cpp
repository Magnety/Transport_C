
#include "gurobi_c++.h"
#include <math.h>
#include <iomanip>

using namespace std;

int main()
{
	int yield[6] = { 1,11,60,9,26,37 };//工厂产量
	int capacity[5] = { 2,25,23,62,32 };//仓库存量
	int p[6][2] = { {18,11},{46,56},{68,82},{88,28},{84,23},{43,60} };//工厂坐标
	int q[5][2] = { {44,36},{35,81},{25,71},{73,79},{78,72} };//仓库坐标
	double dis[6][5] = { 0 };//工厂i和仓库j之间的距离
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 5; j++) {
			dis[i][j] = sqrt((p[i][0]-q[j][0])*(p[i][0] - q[j][0])+ (p[i][1] - q[j][1])*(p[i][1] - q[j][1]));
		}
	}

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 5; j++) {
			printf("%f  ", dis[i][j]);
		}
		printf("\n");
	}

	try {
		GRBEnv env = GRBEnv();//初始化Gurobi的模型
		GRBModel model = GRBModel(env);
		GRBVar** C = 0;//C矩阵为不同工厂需要运输到不同仓库之间的货物量
		C = new GRBVar*[6];
		for (int k = 0; k < 6; ++k)
		{
			C[k] = new GRBVar[5];
			for (int i = 0; i < 10; ++i)
			{
				C[k][i] = model.addVar(0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "x");
			}
		}
		GRBLinExpr lhs = 0;
		for (int i = 0; i < 6; i++) {
			lhs = 0;
			for (int j = 0; j < 5; j++) {
				lhs += C[i][j];
			}
			model.addConstr(lhs==yield[i]);
		}
		GRBLinExpr lhs1 = 0;
		for (int j = 0; j < 5; j++) {
			lhs1 = 0;
			for (int i = 0; i < 6; i++) {
				lhs1 += C[i][j];
			}
			model.addConstr(lhs1 == capacity[j]);
		}

		GRBQuadExpr cost = 0;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 5; j++) {
				cost += C[i][j] * dis[i][j];
			}
		}
		model.setObjective(cost, GRB_MINIMIZE);
		model.optimize();
		printf("The Location and Yield of factories:\n");
		for (int i = 0; i < 6; i++) {
		cout << setw(3) <<"Factory"<<i+1<<"->("<<p[i][0]<<","<<p[i][1]<<")"<<":"<<yield[i];
		printf("\n");
		}
		printf("The Location and Capacity of Warehouses:\n");
		for (int j = 0; j < 5; j++) {
			cout << setw(3) << "Warehouse" << j + 1 << "->(" << q[j][0] << "," << q[j][1] << ")" << ":" << capacity[j];
			printf("\n");
		}
		printf("The Distance between factoris and warehouses:\n");
		cout << setw(10) << " "; 
		for (int j = 0;j < 5; j++) { cout << setw(10) << "Warehouse"<<j+1; }
		printf("\n");
		for (int i = 0; i < 6; i++) {
			cout << setw(10) << "Factory" << i + 1;
			for (int j = 0; j < 5; j++) {
				cout << setw(10) << dis[i][j];
			}
			printf("\n");
		}
		printf("The Transport Matrix:\n");
		cout << setw(10) << " ";
		for (int j = 0; j < 5; j++) { cout << setw(10) << "Warehouse" << j + 1; }printf("\n");
		for (int i = 0; i < 6; i++) {
			cout << setw(10) << "Factory" << i + 1;
			for (int j = 0; j < 5; j++) {
				cout << setw(10)<< C[i][j].get(GRB_DoubleAttr_X);
			}
			printf("\n");
		}
		printf("The total cost:\n");
		cout << "Cost: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

	return 0;
}