#include "def.h"

double Evaluate_Objective(double* current_sol) {
	double objval = 0;
	for (int i = 0; i < N; i++) {
		objval += fixed_cost[i] * current_sol[i];
	}
	objval += GBD_Separator(current_sol);
	return objval;
}

void Heuristic_nonfix() {
	int flag = 1;
	while (flag) {
		flag = close_min_facility_and_nonfix();
	}
	//printf("UpperBound1: %f\n", UpperBound1);
}


int close_min_facility_and_nonfix() {
	int i, plant;

	double valor = 0, MinCostChange = 1;

	for (i = 0; i < N; i++) {
		if (current_sol1[i] == 1 && X[i] < MinCostChange) {
			plant = i;
			MinCostChange = X[i];
		}
	}
	if (MinCostChange == 1)
		return 0;

	//尝试关闭
	current_sol1[plant] = 0;
	valor = Evaluate_Objective(current_sol1);
	if (valor > 0 && valor < UpperBound1) {
		UpperBound1 = valor;
		//printf("Close %d\n", plant + 1);
		return 1;

	}
	else {
		current_sol1[plant] = 1;
		return 0;
	}
}


void MatHeuristic(CPXENVptr env, CPXLPptr lp) {

	int status;
	//圆整阈值
	double threshold = 1e-9;
	//圆整得到启发式初始解
	for (int i = 0; i < N; i++) {
		if (X[i] > threshold) {
			current_sol1[i] = 1;
		}
		else {
			current_sol1[i] = 0;
		}
	}
	UpperBound = Evaluate_Objective(current_sol1);
	UpperBound1 = UpperBound;
	//从current_sol1开始启发式操作 关闭设施不固定变量 得到UpperBound1
	Heuristic_nonfix();
	//比较结果
	UpperBound = UpperBound1;
	current_best_sol = current_sol1;


	CPXLPptr lpcopy = CPXcloneprob(env, lp, &status);
	//将前N个连续变量改为0-1整数变量
	for (int i = 0; i < Num_unfixed; i++) {
		globvarctype[i] = 'B';
		globvarind[i] = i;
	}
	status = CPXchgctype(env, lpcopy, Num_unfixed, globvarind, globvarctype);
	if (status) {
		printf("Unable to chage var type.\n");
		exit(3);
	}


	//CPLEX求解根节点
	CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 1);

	//CPLEX Branch and cut parameters (Not Fine-Tuned yet)
	//CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_ON); //output display
	CPXsetintparam(env, CPX_PARAM_PREIND, 0);// Do not use presolve
	CPXsetintparam(env, CPX_PARAM_MIPSEARCH, CPX_MIPSEARCH_TRADITIONAL); // Turn on traditional search for use with control callbacks 
	CPXsetintparam(env, CPX_PARAM_MIPCBREDLP, CPX_OFF);// Let MIP callbacks work on the original model

	CPXsetintparam(env, CPX_PARAM_THREADS, 1); // Number of threads to use
	CPXsetdblparam(env, CPX_PARAM_EPINT, 0);// The violation tolerance of integrality
	CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9);// The violation tolerance of feasibility
	if (P == 2.0)
		CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-9); // e-optimal solution (%gap): The violation tolerance of optimality

	// 设置callback	
	// usercut callback
	//CPXsetusercutcallbackfunc(env, myusercutcallback, NULL);
	// lazycut callback
	CPXsetlazyconstraintcallbackfunc(env, mylazycutcallback, NULL);

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

	status = CPXaddmipstarts(env, lpcopy, 1, Num_unfixed + 1, beg, globvarind, current_best_sol, effortlevel, NULL);
	if (status)
		fprintf(stderr, "CPX add mip starts failed.\n");


	if (status = CPXmipopt(env, lpcopy)) {
		printf("Unable to optimize the BD reformulation\n");
	}
	
	CPXgetmipobjval(env, lpcopy, &UpperBound);//获取上界
	CPXgetbestobjval(env, lpcopy, &LowerBound);//获取下界
	current_best_sol = current_sol1;
	CPXgetmipx(env, lpcopy, current_best_sol, 0, Num_unfixed);//获取解
	//printf("current_best_obj: %f\n", UpperBound);
	status = CPXfreeprob(env, &lpcopy);
	double gap = fabs(UpperBound - LowerBound) / fabs(UpperBound);
	double 	gaptolerance = 1e-4;
	if (P == 2.0)
		gaptolerance = 1e-9;

	if (gap < gaptolerance) {
		printf("ObjVal: %.6f\nGap: %.6f\nRoot node solving completed！\n", UpperBound, gap);
		Num_unfixed = 0;

		////检验全局整数解的合法性		
		//double objval = 0;
		////加上未消除的变量值成本
		//for (int i = 0; i < N; i++) {
		//	objval += current_best_sol[i] * fixed_cost[i];
		//}
		////apply cut separator
		//objval += GBD_Separator(current_best_sol);
		//printf("Check_MIP_ObjVal: %f\n", objval);

		return;
	}
	else {
		//变量消除测试
		//CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF); //output display	
		reduced_cost_var_elimination(env, lp);
	}
}


void reduced_cost_var_elimination(CPXENVptr env, CPXLPptr lp) {
	int status;
	//重新获取线性规划的reduced cost以及下界
	status = CPXlpopt(env, lp);
	if (status)
		printf("error: CPXlpopt!");
	CPXsolution(env, lp, &status, &LowerBound, X, NULL, NULL, Rc);

	//消除变量
	int ind[1] = { 0 };
	char lu[1] = { 'U' };
	double bd[1] = { 0.0 };
	memset(fixed_plants, -1, N * sizeof(int));//初始化fixed_plants为-1

	//reduced_cost_var_elimination
	int fixed_sum = 0;
	for (int i = 0; i < N; i++) {
		if (Rc[i] >= UpperBound - LowerBound) {
			ind[0] = i;
			lu[0] = 'U';
			bd[0] = 0.0;
			status = CPXchgbds(env, lp, 1, ind, lu, bd);
			fixed_sum += 1;
			fixed_plants[i] = 0;//0代表固定为0
		}
		if (Rc[i] <= LowerBound - UpperBound) {
			ind[0] = i;
			lu[0] = 'L';
			bd[0] = 1.0;
			status = CPXchgbds(env, lp, 1, ind, lu, bd);
			fixed_sum += 1;
			fixed_plants[i] = 1;//1代表固定为1
		}
	}

	//partial_enumeration
	for (int i = 0; i < N; i++) {
		if (fixed_plants[i] == -1) {//从没有固定的里面下手
			if (X[i] < 0.1) {
				ind[0] = i;
				lu[0] = 'L';
				bd[0] = 1.0;
				status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试固定为1
				if (status)
					printf("error: CPXchgbds!");

				if (CPXlpopt(env, lp)) {
					printf("Failed to optimize LP.\n");
					exit(1);
				}
				if (CPXgetobjval(env, lp, &LowerBound_Try)) {
					printf("Failed to getx LP.\n");
				}


				if (LowerBound_Try > UpperBound) {
					bd[0] = 0.0;
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//还原
					if (status)
						printf("error: CPXchgbds!");
					lu[0] = 'U';
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试成功 固定为0
					if (status)
						printf("error: CPXchgbds!");

					fixed_plants[i] = 0;//0代表固定为0
					fixed_sum += 1;
				}
				else {
					bd[0] = 0.0;
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试失败 还原
					if (status)
						printf("error: CPXchgbds!");
				}
			}
			if (X[i] > 0.9) {
				ind[0] = i;
				lu[0] = 'U';
				bd[0] = 0.0;
				status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试固定为0
				if (status)
					printf("error: CPXchgbds!");

				if (CPXlpopt(env, lp)) {
					printf("Failed to optimize LP.\n");
					exit(1);
				}
				if (CPXgetobjval(env, lp, &LowerBound_Try)) {
					printf("Failed to getx LP.\n");
				}

				if (LowerBound_Try > UpperBound) {
					bd[0] = 1.0;
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//还原
					if (status)
						printf("error: CPXchgbds!");
					lu[0] = 'L';
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试成功 固定为1
					if (status)
						printf("error: CPXchgbds!");

					fixed_plants[i] = 1;//1代表固定为1
					fixed_sum += 1;
				}
				else {
					bd[0] = 1.0;
					status = CPXchgbds(env, lp, 1, ind, lu, bd);//尝试失败 还原
					if (status)
						printf("error: CPXchgbds!");
				}

			}
		}
	}


	////检查模型
	//status = CPXwriteprob(env, lp, "master.lp", NULL);
	//if (status) {
	//	fprintf(stderr, "Failed to write LP to disk.\n");
	//}

	Num_unfixed = N - fixed_sum;
	int k = 0;
	for (int i = 0; i < N; i++) {
		if (fixed_plants[i] >= 0) {
			Y[i] = (double)fixed_plants[i];
			pre_fix_sum += fixed_cost[i] * fixed_plants[i];
		}
		else {
			unfixed_index[k] = i;
			k++;
		}
	}


	//修改模型
	double coef[1], rhs[1];
	int indices[1];

	int numConstr = CPXgetnumrows(env, lp);
	for (int i = 0; i < numConstr; i++) {
		if (CPXgetrhs(env, lp, rhs, i, i))
			printf("无法获取rhs");
		for (int j = 0; j < N; j++) {
			if (fixed_plants[j] == 1.0) {
				if (CPXgetcoef(env, lp, i, j, coef))
					printf("无法获取coef");
				rhs[0] -= coef[0];
			}
		}
		indices[0] = i;
		if (CPXchgrhs(env, lp, 1, indices, rhs))
			printf("无法更改rhs");
	}

	//移除所有固定的变量
	int j = 0;
	for (int i = 0; i < N; i++) {
		if (fixed_plants[i] >= 0) {
			if (CPXdelcols(env, lp, i - j, i - j))
				printf("无法删除变量\n");
			else
				j++;
		}
	}

	if (Num_unfixed == 0)
		printf("%d个变量完全消除，求解完成！\n", N);
	else
		printf("Elimination Varaibles: %d\n", fixed_sum);
}