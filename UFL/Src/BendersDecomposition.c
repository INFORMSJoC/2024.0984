#include "def.h"

void BendersDecomposition(void) {
	// 构建主问题模型
	CPXLPptr  lp = NULL;      // data strucutre to store a problem in cplex ...................
	CPXENVptr env = NULL;     // cplex environment.............................................

	//构建主问题
	MasterProblem(&env, &lp);

	// 根节点处理
	Root_node_solve(env, lp);

	if (Num_unfixed > 0) {

		//设置分支定界加割平面算法参数
		SetBranchandCutParam(env, lp);

		//Branch and Benders cut求解
		solve_ip_and_get_solution_info(env, lp);
	}


	/* Free up the problem as allocated by CPXcreateprob, if necessary */
	int status;
	if (lp != NULL) {
		status = CPXfreeprob(env, &lp);
		if (status) {
			fprintf(stderr, "CPXfreeprob failed, error code %d.\n", status);
		}
	}

	/* Free up the CPLEX environment, if necessary */
	if (env != NULL) {
		status = CPXcloseCPLEX(&env);
		if (status) {
			fprintf(stderr, "Could not close CPLEX environment.\n");
			CPXgeterrorstring(env, status, errmsg);
			fprintf(stderr, "%s", errmsg);
		}
	}

}

void MasterProblem(CPXENVptr* envptr, CPXLPptr* lpptr) {
	pre_fix_sum = 0;
	Num_unfixed = N;
	for (int i = 0; i < N; i++) {
		unfixed_index[i] = i;
	}

	int  status = 0;       // optimization status......................... .................
	char probname[16]; // problem name for cplex .......................................
	int numcols = N + 1;//多了一个η变量	

	CPXLPptr  lp;      // data strucutre to store a problem in cplex ...................
	CPXENVptr env;     // cplex environment.............................................

	//Initialize the cplex environment and lp object
	/******************************************************************************************/
	env = CPXopenCPLEX(&status);//cplex自带函数 创建cplex环境
	if (env == NULL) {
		char  errmsg[1024];
		printf("Could not open CPLEX. \n");
		CPXgeterrorstring(env, status, errmsg);
		printf("%s", errmsg);
	}
	// Create the problem in CPLEX 
	strcpy(probname, "qUFL");
	lp = CPXcreateprob(env, &status, probname);//cplex自带函数 建立模型
	if (lp == NULL) {
		fprintf(stderr, "Failed to create LP.\n");
	}

	//局部变量定义
	double* obj = create_double_vector(numcols);    // objective function coefficients of the design variables
	double* lb = create_double_vector(numcols);     //lower bound of the design variables
	double* ub = create_double_vector(numcols);     //upper bound of the design variables
	char** varname = create_stringarray(numcols, 10);//字符串数组 存储变量名	

	//初始化
	for (int i = 0; i < N; i++) {
		obj[i] = fixed_cost[i];//目标函数 决策变量系数
		lb[i] = 0;
		ub[i] = 1;
		sprintf(varname[i], "y_(%d)", i);
	}
	obj[N] = 1;//目标函数 决策变量系数
	lb[N] = 0;
	if (P == -1 || P == -2)
		lb[N] = -CPX_INFBOUND;
	ub[N] = CPX_INFBOUND;
	sprintf(varname[N], "eta_");

	//cplex添加变量
	status = CPXnewcols(env, lp, numcols, obj, lb, ub, NULL, varname);
	if (status) goto QUIT;
	free_stringarray(varname, numcols);
	free(obj);
	free(lb);
	free(ub);

	//Initializing the master problem constraints
	/**********************************************/
	int	numrows = 1;  // number of constraints.........................................
	int	numnz = N;   // number of non-zero elements in the matrix ....................
	//局部变量定义
	double* rhs = create_double_vector(numrows);
	char* sense = create_char_vector(numrows);
	int* matbeg = create_int_vector(numrows);
	int* matind = create_int_vector(numnz);
	double* matval = create_double_vector(numnz);

	//初始化	
	sense[0] = 'G';
	rhs[0] = 2;
	matbeg[0] = 0;
	for (int i = 0; i < N; i++) {
		matind[i] = i;
		matval[i] = 1;
	}
	//cpxaddrows(env, lp, 新增变量个数, 新增约束个数, 非零约束系数个数, 右端项, 不等式方向, 约束矩阵中约束开始的索引, 约束矩阵索引, 约束矩阵索引所对应的非零系数值, 新增变量名称, 新增约束名称)
	status = CPXaddrows(env, lp, 0, numrows, numnz, rhs, sense, matbeg, matind, matval, NULL, NULL);
	if (status) goto QUIT;
	free(rhs);
	free(sense);
	free(matbeg);
	free(matind);
	free(matval);
QUIT:
	//修改env, lp数据
	*envptr = env;
	*lpptr = lp;
}

void Root_node_solve(CPXENVptr env, CPXLPptr lp) {
	int status;
	clock_t start, end;//clock_t是记录时间的数据类型，在time.h中定义
	//设置CPLEX求解参数
	CPXsetintparam(env, CPX_PARAM_THREADS, 1);
	start = clock();
	printf("\n-----------------------------------------------------------------------------------------------------------------------\nSolving root node\n");
	if (CPXlpopt(env, lp)) {//求解线性规划问题
		printf("Failed to optimize LP.\n");
		exit(2);
	}
	//获取线性规划的最优解 
	CPXsolution(env, lp, &status, &LowerBound, X, NULL, NULL, NULL);

	//Stabilization
	for (int i = 0; i < N; i++) {
		Stabilizing_Point[i] = 1;
	}
	double lambda = 0.8;
	Cut_Stabilization(env, lp, lambda);


	//使用数学启发式在根节点获得高质量上界
	MatHeuristic(env, lp);

	end = clock();

	////检查模型
	//status = CPXwriteprob(env, lp, "master.lp", NULL);
	//if (status) {
	//	fprintf(stderr, "Failed to write LP to disk.\n");
	//}

	// 换算cpu时间为s
	RootRuntime = ((double)end - (double)start) / CLOCKS_PER_SEC;
	printf("RootRuntime: %f\n", RootRuntime);

	//根节点结果写入到文件
	if (Num_unfixed == 0) {
		fprintf(output, "RootUpperBound:%.6f;RootLowerBound:%.6f;RootGap:%.4f;RootRuntime:%.4f;\n", UpperBound, LowerBound, fabs(UpperBound - LowerBound) / fabs(UpperBound), RootRuntime);
		fprintf(output, "Objval:%.6f;Gap:%.4f;TotalRuntime:%.4f;Nodes:%d;Status:%d;\n", UpperBound, fabs(UpperBound - LowerBound) / fabs(UpperBound), RootRuntime, 0, 102);
	}
	else
		fprintf(output, "RootUpperBound:%.6f;RootLowerBound:%.6f;RootGap:%.4f;RootRuntime:%.4f;\n", UpperBound, LowerBound, fabs(UpperBound - LowerBound) / fabs(UpperBound), RootRuntime);


	printf("Finished solving root node\n-----------------------------------------------------------------------------------------------------------------------\n");

}

void Cut_Stabilization(CPXENVptr env, CPXLPptr lp, double lambda) {
	int status, terminate = 0;
	double old_objval = LowerBound, phi;
	do {
		//更新稳定点
		for (int i = 0; i < N; i++) {
			Stabilizing_Point[i] = 0.5 * Stabilizing_Point[i] + 0.5 * X[i];
		}

		//更新分离点
		for (int i = 0; i < N; i++) {
			Separation_Point[i] =  lambda * X[i] + (1 - lambda) * Stabilizing_Point[i];
		}

		// Solve the subproblem
		phi = GBD_Separator(Separation_Point);

		//If a violated cut is found, then solve the LP again
		status = CPXaddrows(env, lp, 0, 1, Num_unfixed + 1, benders_cut->rhs, benders_cut->sense, benders_cut->beg, benders_cut->ind, benders_cut->val, NULL, NULL);
		if (status) {
			printf("Unable to add Benders cut.\n");
			exit(3);
		}
	
		if (CPXlpopt(env, lp)) {
			printf("Failed to optimize LP.\n");
			exit(1);
		}
		CPXsolution(env, lp, &status, &LowerBound, X, NULL, NULL, NULL);

		//printf("LowerBound: %.6f \n", pre_fix_sum + LowerBound);

		//Should there be a second round or not?
		if (abs(LowerBound - old_objval) > EPS) {
			terminate = 0;
			old_objval = LowerBound;
		}
		else {
			terminate += 1;
		}

	} while (terminate <= 5);
}

int CPXPUBLIC mylazycutcallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p) {
	*useraction_p = CPX_CALLBACK_DEFAULT;
	// get solution x
	int status = CPXgetcallbacknodex(env, cbdata, wherefrom, X, 0, Num_unfixed);
	if (status) {
		fprintf(stderr, "Failed to get node solution.\n");
	}
	//apply cut separator
	double phi = GBD_Separator(X);

	//add violated cuts
	status = CPXcutcallbackadd(env, cbdata, wherefrom, Num_unfixed + 1, benders_cut->rhs[0], benders_cut->sense[0], benders_cut->ind, benders_cut->val, CPX_USECUT_FORCE);//CPX_USECUT_PURGE
	if (status) {
		fprintf(stderr, "Failed to add cut.\n");
	}

	// tell CPLEX that cuts have been created*/
	*useraction_p = CPX_CALLBACK_SET;
	return 0;
}

int CPXPUBLIC myusercutcallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p) {
	*useraction_p = CPX_CALLBACK_DEFAULT;
	// get solution x
	int status = CPXgetcallbacknodex(env, cbdata, wherefrom, X, 0, Num_unfixed);
	if (status) {
		fprintf(stderr, "Failed to get node solution.\n");
	}
	//apply cut separator
	double phi = GBD_Separator(X);

	//add violated cuts
	if (X[Num_unfixed] + EPS < phi) {
		//status = CPXcutcallbackaddlocal(env, cbdata, wherefrom, Num_unfixed + 1, benders_cut->rhs[0], benders_cut->sense[0], benders_cut->ind, benders_cut->val, CPX_USECUT_FORCE);
		status = CPXcutcallbackadd(env, cbdata, wherefrom, Num_unfixed + 1, benders_cut->rhs[0], benders_cut->sense[0], benders_cut->ind, benders_cut->val, CPX_USECUT_FORCE);
		if (status) {
			fprintf(stderr, "Failed to add cut.\n");
		}
	}

	//避免拖尾现象
	// Get the Node Id
	CPXgetcallbacknodeinfo(env, cbdata, wherefrom, 0, CPX_CALLBACK_INFO_NODE_SEQNUM, &node_id);

	// Remember the Node Id
	if (last_id == node_id)
		last_id_visits++;
	else {
		last_id = node_id;
		last_id_visits = 0;
	}

	if (last_id_visits > 10)
		*useraction_p = CPX_CALLBACK_ABORT_CUT_LOOP;
	else
		*useraction_p = CPX_CALLBACK_DEFAULT;
	return 0;
}

void SetBranchandCutParam(CPXENVptr env, CPXLPptr lp) {
	int status;
	//将前N个连续变量改为0-1整数变量
	for (int i = 0; i < Num_unfixed; i++) {
		globvarctype[i] = 'B';
		globvarind[i] = i;
	}
	status = CPXchgctype(env, lp, Num_unfixed, globvarind, globvarctype);
	if (status) {
		printf("Unable to chage var type.\n");
		exit(3);
	}

	//开始分支节点求解
	CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 999999999);
	CPXsetdblparam(env, CPX_PARAM_TILIM, 1000 - RootRuntime); // time limit
	//output display
	//CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_ON);

	// Add best solution from Matheuristic to MIP
	int beg[1], effortlevel[1];
	beg[0] = 0;
	effortlevel[0] = 5;
	globvarind[Num_unfixed] = Num_unfixed;
	current_best_sol[Num_unfixed] = UpperBound - pre_fix_sum;
	for (int i = 0; i < Num_unfixed; i++) {
		current_best_sol[i] = current_best_sol[unfixed_index[i]];
		current_best_sol[Num_unfixed] -= current_best_sol[unfixed_index[i]] * fixed_cost[unfixed_index[i]];
	}

	status = CPXaddmipstarts(env, lp, 1, Num_unfixed + 1, beg, globvarind, current_best_sol, effortlevel, NULL);
	if (status)
		fprintf(stderr, "CPX add mip starts failed.\n");
}



void solve_ip_and_get_solution_info(CPXENVptr env, CPXLPptr lp) {
	int status, nodecount;
	clock_t start, end;//clock_t是记录时间的数据类型，在time.h中定义
	double NodesRuntime, objval = 0, gap;

	printf("\n-----------------------------------------------------------------------------------------------------------------------\nEntering Branch and Cut\n");
	//solve the ip
	start = clock();
	if (status = CPXmipopt(env, lp)) {
		printf("Unable to optimize the BD reformulation\n");
	}
	end = clock();

	//Get the status and classify accordingly
	status = CPXgetstat(env, lp);
	switch (status) {
	case 101:
		printf("Optimal solution found!\n");
		break;
	case 102:
		printf("Epsilon-Optimal solution found!\n");
		break;
	case 103:
		printf("Master Problem infeasible!\n");
		break;
	case 105:
	case 106:
	case 107:
	case 108:
		printf("Time limit reached!\n");
		break;
	case 115:
		printf("Problem is optimal with unscaled infeasibilities!\n");
		break;
	default:		
		CPXgeterrorstring(env, status, errmsg);
		fprintf(stderr, "%s", errmsg);
	}


	//collect and store the information
	NodesRuntime = ((double)end - (double)start) / CLOCKS_PER_SEC;
	if (status != 103) {
		CPXgetmipobjval(env, lp, &objval);
		CPXgetmiprelgap(env, lp, &gap);
		nodecount = CPXgetnodecnt(env, lp);
		printf("MIP_ObjVal: %f\n", objval + pre_fix_sum);
		printf("MIP_Gap: %f\n", gap);
		printf("MIP_Nodes: %d\n", nodecount);
		printf("MIP_Runtime: %f\n", NodesRuntime);

		//写入到文件
		fprintf(output, "Objval:%.6f;Gap:%.4f;TotalTime:%.4f;Nodes:%d;Status:%d;\n", objval + pre_fix_sum, gap, RootRuntime + NodesRuntime, nodecount, status);
	}


	////检验全局整数解的合法性
	//CPXgetmipx(env, lp, X, 0, Num_unfixed);//获取解
	////加上已经消除的变量值成本
	//objval = 0;
	//objval += pre_fix_sum;
	////加上未消除的变量值成本
	//for (int i = 0; i < Num_unfixed; i++) {
	//	objval += X[i] * fixed_cost[unfixed_index[i]];
	//}
	////apply cut separator
	//objval += GBD_Separator(X);
	//printf("Check_MIP_ObjVal: %f\n", objval);

	printf("Finished the Branch and Cut\n-----------------------------------------------------------------------------------------------------------------------\n");
}

double GBD_Separator(double* Z) {
	for (int i = 0; i < Num_unfixed; i++) {
		Y[unfixed_index[i]] = Z[i];
	}

	benders_cut->rhs[0] = 0;//每次分离cut要求的
	memset(benders_cut->val, 0, (N + 1) * sizeof(double));//每次分离cut要求的

	double voptsum = 0;
	for (int j = 0; j < M; j++) {
		double voptj = 0, gramma = 0, lambda = 0, rhs = 1;
		//求解归约后的问题
		if (P == 2.0) {//针对x^2			
			for (int i = 0; i < N; i++) {
				Rlist[i] = i;
				if (Y[i] == 0) {
					x[i] = 0;
					Rlist[i] = N;//当Rlist[i]的值等于n可视为该元素删除
				}
				else {
					a[i] = 0.5 * Y[i] / quad_cost[i][j];//归约问题系数
					gramma += a[i];
				}
			}
			lambda = rhs / gramma;

			bool terminate = false;
			double delta = 0;
			//循环处理
			while (terminate == false) {//KC算法
				terminate = true;
				delta = 0;
				for (int i = 0; i < N; i++) {
					if (Rlist[i] < N) {//当Rlist[i]的值等于n可视为该元素删除
						x[i] = lambda * a[i];
						if (x[i] >= Y[i]) {
							delta += x[i] - Y[i];
							gramma -=  a[i];
							x[i] = Y[i];
							Rlist[i] = N;//意味着被删除
							terminate = false;
						}
					}
				}
				if (terminate == false)
					lambda += delta / gramma;
			}

			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					alpha[i] = lambda;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda - quad_cost[i][j] * P);
					if (x[i] == Y[i])
						subgradient[i] = -alpha[i] - quad_cost[i][j];					
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] - quad_cost[i][j] * x_[i] * x_[i];
					}
					voptj += 0.5 / a[i] * x[i] * x[i];
				}

			}
			voptsum += voptj;
		}

		if (P == 1.5) {//这个函数针对cx^p	
			for (int i = 0; i < N; i++) {
				Rlist[i] = i;
				if (Y[i] == 0) {
					x[i] = 0;
					Rlist[i] = N;//当Rlist[i]的值等于n可视为该元素删除
				}
				else {
					a[i] = Y[i] / (quad_cost[i][j] * quad_cost[i][j]);//归约问题系数
					gramma += a[i];
				}
			}
			lambda = rhs / gramma;

			bool terminate = false;
			double delta = 0;
			//循环处理
			while (terminate == false) {
				terminate = true;
				delta = 0;
				for (int i = 0; i < N; i++) {
					if (Rlist[i] < N) {//当Rlist[i]的值等于n可视为该元素删除
						x[i] = lambda * a[i];
						if (x[i] >= Y[i]) {
							delta += x[i] - Y[i];
							gramma -= a[i];
							x[i] = Y[i];
							Rlist[i] = N;//意味着被删除
							terminate = false;
						}
					}
				}
				lambda += delta / gramma;
			}

			int flag = 0;
			for (int i = 0; i < N; i++) {
				if (Rlist[i] < N) {
					lambda = quad_cost[i][j] * P * sqrt(x[i] / Y[i]);
					flag = 1;
					break;
				}
			}
			if (flag == 0)
				lambda = 0;

			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					alpha[i] = lambda;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda - quad_cost[i][j] * P);
					if (x[i] == Y[i]) {
						subgradient[i] = -alpha[i] + (1 - P) * quad_cost[i][j];
						voptj += quad_cost[i][j] * Y[i];
					}
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] - quad_cost[i][j] * 0.5 * sqrt(x_[i]) * x_[i];
						voptj += quad_cost[i][j] * Y[i] * sqrt(x_[i]) * x_[i];
					}
				}
			}

			voptsum += voptj;
		}

		if (P == 2.5) {//这个函数针对cx^p	
			for (int i = 0; i < N; i++) {
				Rlist[i] = i;
				if (Y[i] == 0) {
					x[i] = 0;
					Rlist[i] = N;//当Rlist[i]的值等于n可视为该元素删除
				}
				else {
					a[i] = Y[i] / pow(quad_cost[i][j], 1 / (P - 1));//归约问题系数
					gramma += a[i];
				}
			}
			lambda = rhs / gramma;

			bool terminate = false;
			double delta = 0;
			//循环处理
			while (terminate == false) {
				terminate = true;
				delta = 0;
				for (int i = 0; i < N; i++) {
					if (Rlist[i] < N) {//当Rlist[i]的值等于n可视为该元素删除
						x[i] = lambda * a[i];
						if (x[i] >= Y[i]) {
							delta += x[i] - Y[i];
							gramma -= a[i];
							x[i] = Y[i];
							Rlist[i] = N;//意味着被删除
							terminate = false;
						}
					}
				}
				lambda += delta / gramma;
			}

			int flag = 0;
			for (int i = 0; i < N; i++) {
				if (Rlist[i] < N) {
					lambda = quad_cost[i][j] * P * sqrt(x[i] / Y[i]) * (x[i] / Y[i]);
					flag = 1;
					break;
				}
			}
			if (flag == 0)
				lambda = 0;

			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					alpha[i] = lambda;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda - quad_cost[i][j] * P);
					if (x[i] == Y[i]) {
						subgradient[i] = -alpha[i] + (1 - P) * quad_cost[i][j];
						voptj += quad_cost[i][j] * Y[i];
					}
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] - quad_cost[i][j] * 1.5 * sqrt(x_[i]) * x_[i] * x_[i];
						voptj += quad_cost[i][j] * Y[i] * sqrt(x_[i]) * x_[i] * x_[i];
					}
				}
			}

			voptsum += voptj;
		}

		if (P == 3.0) {//这个函数针对cx^p	
			for (int i = 0; i < N; i++) {
				Rlist[i] = i;
				if (Y[i] == 0) {
					x[i] = 0;
					Rlist[i] = N;//当Rlist[i]的值等于n可视为该元素删除
				}
				else {
					a[i] = Y[i] / sqrt(quad_cost[i][j]);//归约问题系数
					gramma += a[i];
				}
			}
			lambda = rhs / gramma;

			bool terminate = false;
			double delta = 0;
			//循环处理
			while (terminate == false) {
				terminate = true;
				delta = 0;
				for (int i = 0; i < N; i++) {
					if (Rlist[i] < N) {//当Rlist[i]的值等于n可视为该元素删除
						x[i] = lambda * a[i];
						if (x[i] >= Y[i]) {
							delta += x[i] - Y[i];
							gramma -= a[i];
							x[i] = Y[i];
							Rlist[i] = N;//意味着被删除
							terminate = false;
						}
					}
				}
				lambda += delta / gramma;
			}

			int flag = 0;
			for (int i = 0; i < N; i++) {
				if (Rlist[i] < N) {
					lambda = quad_cost[i][j] * P * (x[i] / Y[i]) * (x[i] / Y[i]);
					flag = 1;
					break;
				}
			}
			if (flag == 0)
				lambda = 0;

			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					alpha[i] = lambda;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda - quad_cost[i][j] * P);
					if (x[i] == Y[i]) {
						subgradient[i] = -alpha[i] + (1 - P) * quad_cost[i][j];
						voptj += quad_cost[i][j] * Y[i];
					}
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] - quad_cost[i][j] * 2 * x_[i] * x_[i] * x_[i];
						voptj += quad_cost[i][j] * Y[i] * x_[i] * x_[i] * x_[i];
					}
				}
			}

			voptsum += voptj;
		}

		if (P == 0) {//这个函数针对归约后的二次优化问题 Kleinrock average delay function
			memset(S, -1, sizeof(int) * N);//全部初始化为-1
			double temp = 0;
			for (int i = 0; i < N; i++) {
				if (Y[i] == 0)
					x[i] = 0; //S[i]等于默认值-1意味着被删除
				else {
					a[i] = sqrt(quad_cost[i][j]) * Y[i];//获取归约后的二次资源分配问题的参数a[]和b[]
					b[i] = -1 / sqrt(quad_cost[i][j]);//获取归约后的二次资源分配问题的参数a[]和b[]

					S[i] = i;//正常初始化
					gramma += a[i];
					temp += b[i] * a[i];
				}
			}
			lambda = (temp + rhs) / gramma;

			double delta1, delta2;
			int index, k;
			while (1) {
				//归零
				delta1 = 0, delta2 = 0;
				memset(S1, -1, sizeof(int) * N), memset(S2, -1, sizeof(int) * N);
				index = 0, k = 0;

				for (int i = 0; i < N; i++) {
					if (S[i] != -1) {
						x[i] = (lambda - b[i]) * a[i];
						if (x[i] <= 0) {
							delta1 -= x[i];
							S1[index] = i;
							index += 1;
						}

						if (x[i] >= Y[i]) {
							delta2 += (x[i] - Y[i]);
							S2[k] = i;
							k += 1;
						}
					}
				}

				if (delta1 == delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					break;
				}
				else if (delta1 > delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S1[i]];
						}
					}
					lambda -= delta1 / gramma;
				}
				else {
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S2[i]];
						}
					}
					lambda += delta2 / gramma;
				}
			}

			//计算S是否为空集, 以获取原问题的对偶乘子
			int flag = 1;
			for (int i = 0; i < N; i++) {
				if (S[i] != -1) {
					flag = 0;
					x_[i] = x[i] / Y[i];
					lambda = quad_cost[i][j] / ((1 - x_[i]) * (1 - x_[i]));//对原问题的透视函数进行修改
					break;
				}
			}
			if (flag == 1)
				lambda = 0;


			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					//alpha[i] = lambda;
					alpha[i] = MAX(0, lambda - quad_cost[i][j]);//修改bug;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = 0;//对原问题的透视函数进行修改			
					x_[i] = x[i] / Y[i];
					subgradient[i] = -alpha[i]  - quad_cost[i][j] * x_[i] * x_[i] / ((1 - x_[i]) * (1 - x_[i]));//对原问题的透视函数进行修改
					voptj += quad_cost[i][j] * x[i] / (1 - x_[i]);//对原问题的透视函数进行修改
				}

			}

			voptsum += voptj;
		}

		if (P == -1) {//这个函数针对归约后的二次优化问题 ce^(-x)
			memset(S, -1, sizeof(int) * N);//全部初始化为-1
			double temp = 0;
			for (int i = 0; i < N; i++) {
				if (Y[i] == 0)
					x[i] = 0; //S[i]等于默认值-1意味着被删除
				else {
					a[i] = Y[i];
					b[i] = -log(quad_cost[i][j]);//获取归约后的二次资源分配问题的参数a[]和b[]

					S[i] = i;//正常初始化
					gramma += a[i];
					temp += b[i] * a[i];
				}
			}
			lambda = (temp + rhs) / gramma;

			double delta1, delta2;
			int index, k;
			while (1) {
				//归零
				delta1 = 0, delta2 = 0;
				memset(S1, -1, sizeof(int) * N), memset(S2, -1, sizeof(int) * N);
				index = 0, k = 0;

				for (int i = 0; i < N; i++) {
					if (S[i] != -1) {
						x[i] = (lambda - b[i]) * a[i];
						if (x[i] <= 0) {
							delta1 -= x[i];
							S1[index] = i;
							index += 1;
						}

						if (x[i] >= Y[i]) {
							delta2 += (x[i] - Y[i]);
							S2[k] = i;
							k += 1;
						}
					}
				}

				if (delta1 == delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					break;
				}
				else if (delta1 > delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S1[i]];
						}
					}
					lambda -= delta1 / gramma;
				}
				else {
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S2[i]];
						}
					}
					lambda += delta2 / gramma;
				}
			}

			//计算S是否为空集, 以获取原问题的对偶乘子
			int flag = 1;
			for (int i = 0; i < N; i++) {
				if (S[i] != -1) {
					flag = 0;
					x_[i] = x[i] / Y[i];
					lambda = -quad_cost[i][j] * exp(-x_[i]);//对原问题的透视函数进行修改
					break;
				}
			}
			if (flag == 1)
				lambda = 0;


			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					//alpha[i] = lambda;
					alpha[i] = MAX(0, lambda + quad_cost[i][j]);//修改bug;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda + quad_cost[i][j] * exp(-1) );//对原问题的透视函数进行修改
					if (x[i] == Y[i]) {
						subgradient[i] = -alpha[i] + quad_cost[i][j] * (2 * exp(-1) - 1);//对原问题的透视函数进行修改
						voptj += quad_cost[i][j] * Y[i] * (exp(-1) - 1);//对原问题的透视函数进行修改
					}
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] + quad_cost[i][j] * (exp(-x_[i]) - 1 + x_[i] * exp(-x_[i]));//对原问题的透视函数进行修改
						voptj += quad_cost[i][j] * Y[i] * (exp(-x_[i]) - 1);//对原问题的透视函数进行修改
					}
				}
			}

			voptsum += voptj;
		}

		if (P == -2) {//这个函数针对归约后的二次优化问题 - clog(1+x)
			memset(S, -1, sizeof(int) * N);//全部初始化为-1
			double temp = 0;
			for (int i = 0; i < N; i++) {
				if (Y[i] == 0)
					x[i] = 0; //S[i]等于默认值-1意味着被删除
				else {
					a[i] = quad_cost[i][j] * Y[i];
					b[i] = 1 / quad_cost[i][j];//获取归约后的二次资源分配问题的参数a[]和b[]

					S[i] = i;//正常初始化
					gramma += a[i];
					temp += b[i] * a[i];
				}
			}
			lambda = (temp + rhs) / gramma;

			double delta1, delta2;
			int index, k;
			while (1) {
				//归零
				delta1 = 0, delta2 = 0;
				memset(S1, -1, sizeof(int) * N), memset(S2, -1, sizeof(int) * N);
				index = 0, k = 0;

				for (int i = 0; i < N; i++) {
					if (S[i] != -1) {
						x[i] = (lambda - b[i]) * a[i];
						if (x[i] <= 0) {
							delta1 -= x[i];
							S1[index] = i;
							index += 1;
						}

						if (x[i] >= Y[i]) {
							delta2 += (x[i] - Y[i]);
							S2[k] = i;
							k += 1;
						}
					}
				}

				if (delta1 == delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
						}
					}
					break;
				}
				else if (delta1 > delta2) {
					for (int i = 0; i < N; i++) {
						if (S1[i] == -1)
							break;
						else {
							x[S1[i]] = 0;
							S[S1[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S1[i]];
						}
					}
					lambda -= delta1 / gramma;
				}
				else {
					for (int i = 0; i < N; i++) {
						if (S2[i] == -1)
							break;
						else {
							x[S2[i]] = Y[S2[i]];
							S[S2[i]] = -1;//S[i]等于默认值-1意味着被删除
							gramma -= a[S2[i]];
						}
					}
					lambda += delta2 / gramma;
				}
			}

			//计算S是否为空集, 以获取原问题的对偶乘子
			int flag = 1;
			for (int i = 0; i < N; i++) {
				if (S[i] != -1) {
					flag = 0;
					x_[i] = x[i] / Y[i];
					lambda = -quad_cost[i][j] / (1 + x_[i]);//对原问题的透视函数进行修改
					break;
				}
			}
			if (flag == 1)
				lambda = 0;


			for (int i = 0; i < N; i++) {
				if (Y[i] == 0) {
					//alpha[i] = lambda;
					alpha[i] = MAX(0, lambda + quad_cost[i][j]);//修改bug;
					subgradient[i] = -alpha[i];
				}
				else {
					alpha[i] = MAX(0, lambda + quad_cost[i][j] / 2 );//对原问题的透视函数进行修改
					if (x[i] == Y[i]) {
						subgradient[i] = -alpha[i] - quad_cost[i][j] * log(2) + quad_cost[i][j] / 2;//对原问题的透视函数进行修改
						voptj -= quad_cost[i][j] * Y[i] * log(2);//对原问题的透视函数进行修改
					}
					else {
						x_[i] = x[i] / Y[i];
						subgradient[i] = -alpha[i] - quad_cost[i][j] * log(1 + x_[i]) + quad_cost[i][j] * x_[i] / (1 + x_[i]);//对原问题的透视函数进行修改
						voptj -= quad_cost[i][j] * Y[i] * log(1 + x_[i]);//对原问题的透视函数进行修改
					}
				}
			}

			voptsum += voptj;
		}

		//提取未固定变量的对偶信息
		double subgradient_sum = 0;
		for (int i = 0; i < Num_unfixed; i++) {
			subgradient_sum += subgradient[unfixed_index[i]] * Y[unfixed_index[i]];
		}
		benders_cut->rhs[0] += voptj - subgradient_sum;


		for (int i = 0; i < Num_unfixed; i++) {
			benders_cut->val[i] -= subgradient[unfixed_index[i]];
		}

	}
	benders_cut->val[Num_unfixed] = 1;


	return voptsum;
}