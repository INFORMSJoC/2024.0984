#include "def.h"

void PC() {
    clock_t start, end;
    int status, nodecount;
    double NodesRuntime, objval, gap;
    UpperBound = CPX_INFBOUND;

    CPXENVptr env = NULL;
    CPXLPptr  lp = NULL;

    /* Initialize the CPLEX environment */
    env = CPXopenCPLEX(&status);

    /* If an error occurs, the status value indicates the reason for
       failure.  A call to CPXgeterrorstring will produce the text of
       the error message.  Note that CPXopenCPLEX produces no output,
       so the only way to see the cause of the error is to use
       CPXgeterrorstring.  For other CPLEX routines, the errors will
       be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

    if (env == NULL) {
        fprintf(stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring(env, status, errmsg);
        fprintf(stderr, "%s", errmsg);
    }

    /* Turn on output to the screen */
    status = CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_ON);
    if (status) {
        fprintf(stderr,
            "Failure to turn on screen indicator, error %d.\n", status);
    }

    // Create the problem in CPLEX 
    char probname[16];
    strcpy(probname, "SOCP");
    lp = CPXcreateprob(env, &status, probname);

    /* A returned pointer of NULL may mean that not enough memory
       was available or there was some other problem.  In the case of
       failure, an error message will have been written to the error
       channel from inside CPLEX.  In this example, the setting of
       the parameter CPXPARAM_ScreenOutput causes the error message to
       appear on stdout.  */
    if (lp == NULL) {
        fprintf(stderr, "Failed to create LP.\n");
    }

    status = CPXchgobjsen(env, lp, CPX_MIN);  /* Problem is minimization */
    if (status)
        fprintf(stderr, "CPXchgobjsen failed.\n");

   
    int numcols = N + 2 * N * M;
    double* obj = create_double_vector(numcols);
    double* lb = create_double_vector(numcols);
    double* ub = create_double_vector(numcols);
    char* xctype = create_char_vector(numcols);
    char** colname = create_stringarray(numcols, 10);

    for (int i = 0; i < N; i++) {
        obj[i] = fixed_cost[i];
        lb[i] = 0;
        ub[i] = 1;
        xctype[i] = 'B';
        sprintf(colname[i], "y_(%d)", i);

        for (int j = 0; j < M; j++) {
            obj[varindex(i, j) + N] = 0.0;
            lb[varindex(i, j) + N] = 0;
            ub[varindex(i, j) + N] = 1;
            xctype[varindex(i, j) + N] = 'C';
            sprintf(colname[varindex(i, j) + N], "x_(%d,%d)", i, j);

            obj[varindex(i, j) + N * M + N] = 1;
            lb[varindex(i, j) + N * M + N] = 0;
            ub[varindex(i, j) + N * M + N] = CPX_INFBOUND;
            xctype[varindex(i, j) + N * M + N] = 'C';
            sprintf(colname[varindex(i, j) + N * M + N], "z_(%d,%d)", i, j);
        }
    }

    status = CPXnewcols(env, lp, numcols, obj, lb, ub, xctype, colname);
    if (status)
        fprintf(stderr, "CPXnewcols failed.\n");
    free(obj);
    free(lb);
    free(ub);
    free(xctype);
    free(colname);

    int numrows = N * M + M;
    int numnz = 2 * N * M + N * M;
    double* rhs = create_double_vector(numrows);
    char* sense = create_char_vector(numrows);
    char** rowname = create_stringarray(numrows, 10);
    int* rmatbeg = create_int_vector(numrows);
    int* rmatind = create_int_vector(numnz);
    double* rmatval = create_double_vector(numnz);

    for (int j = 0; j < M; j++) {
        for (int i = 0; i < N; i++) {
            rmatbeg[j * N + i] = 2 * j * N + 2 * i;

            rmatind[2 * j * N + 2 * i] = i;
            rmatval[2 * j * N + 2 * i] = 1.0;
            rmatind[2 * j * N + 2 * i + 1] = N + varindex(i, j);
            rmatval[2 * j * N + 2 * i + 1] = -1.0;

            rhs[j * N + i] = 0;
            sense[j * N + i] = 'G';
            sprintf(rowname[j * N + i], "link(%d,%d)", i, j);
        }
    }

    for (int j = 0; j < M; j++) {
        rmatbeg[N * M + j] = 2 * N * M + (j * N);
        for (int i = 0; i < N; i++) {
            rmatind[2 * N * M + (j * N) + i] = N + varindex(i, j);
            rmatval[2 * N * M + (j * N) + i] = 1.0;
        }
        rhs[N * M + j] = 1;
        sense[N * M + j] = 'E';
        sprintf(rowname[N * M + j], "con(%d)", j);
    }
    
    status = CPXaddrows(env, lp, 0, numrows, numnz, rhs, sense, rmatbeg, rmatind, rmatval, NULL, rowname);
    if (status)
        fprintf(stderr, "CPXaddrows failed.\n");
    free(rhs);
    free(sense);
    free(rowname);
    free(rmatbeg);
    free(rmatind);
    free(rmatval);

    ////添加边界的perspective cut
    //double frac = 1.0;
    //char cutsense = 'G';
    //double cutrhs = 0.0;
    //int cutbeg = 0;
    //int cutind[] = { 0, 0, 0 };
    //double cutval[] = { 0.0, 0.0, 1.0 };
    //for (int j = 0; j < M; j++) {
    //    for (int i = 0; i < N; i++) {
    //        //cutind[0] = i;
    //        //cutval[0] = quad_cost[i][j];

    //        //cutind[1] = N + varindex(i, j);
    //        //cutval[1] = -2 * quad_cost[i][j];

    //        //cutind[2] = varindex(i, j) + N * M + N;
    //        ////添加单条perspective cut
    //        //status = CPXaddrows(env, lp, 0, 1, 3, &cutrhs, &cutsense, &cutbeg, cutind, cutval, NULL, NULL);
    //        //if (status) {
    //        //    fprintf(stderr, "CPXaddrow failed.\n");
    //        //}

    //        //随机采样
    //        frac = (double)rand() / RAND_MAX*0.1;
    //        cutind[0] = i;
    //        cutval[0] = quad_cost[i][j] * frac* frac;

    //        cutind[1] = N + varindex(i, j);
    //        cutval[1] = -2 * quad_cost[i][j] * frac;

    //        cutind[2] = varindex(i, j) + N * M + N;
    //        //添加单条perspective cut
    //        status = CPXaddrows(env, lp, 0, 1, 3, &cutrhs, &cutsense, &cutbeg, cutind, cutval, NULL, NULL);
    //        if (status) {
    //            fprintf(stderr, "CPXaddrow failed.\n");
    //        }
    //    }
    //}


    //检查模型
    status = CPXwriteprob(env, lp, "PC.lp", NULL);
    if (status)
        fprintf(stderr, "Failed to write LP to disk.\n");

    //CPLEX Branch and cut parameters (Not Fine-Tuned yet)
    //CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_ON); //output display
    //CPXsetintparam(env, CPX_PARAM_PREIND, 0);// Do not use presolve
    //CPXsetintparam(env, CPX_PARAM_MIPSEARCH, CPX_MIPSEARCH_TRADITIONAL); // Turn on traditional search for use with control callbacks 
    //CPXsetintparam(env, CPX_PARAM_MIPCBREDLP, CPX_OFF);// Let MIP callbacks work on the original model
    CPXsetintparam(env, CPX_PARAM_THREADS, 1); // Number of threads to use
    CPXsetdblparam(env, CPX_PARAM_EPINT, 0);// The violation tolerance of integrality
    CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9);// The violation tolerance of feasibility
 
    // 设置现代callback 添加perspective cut
    // 注册回调函数id为会在候选解和松弛两个阶段执行	
    CPXLONG contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION; 
    // 注册回调函数
    CPXcallbacksetfunc(env, lp, contextid, PC_callback, NULL);



    ////求解根节点松弛解
    //CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 1);
    //start = clock();
    //if (status = CPXmipopt(env, lp)) {
    //    printf("Unable to optimize the PC reformulation\n");
    //}
    //end = clock();
    //RootRuntime = ((double)end - (double)start) / CLOCKS_PER_SEC;
    //CPXgetmipobjval(env, lp, &UpperBound);//获取上界
    //CPXgetmiprelgap(env, lp, &gap);//获取gap
    //CPXgetbestobjval(env, lp, &LowerBound);//获取下界
    ////根节点结果写入到文件
    //fprintf(output, "RootUpperBound:%.6f;RootLowerBound:%.6f;RootGap:%.4f;RootRuntime:%.4f;\n", UpperBound, LowerBound, gap, RootRuntime);

    ////开始分支节点求解
    //CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 999999999);
    CPXsetdblparam(env, CPX_PARAM_TILIM, 1000- RootRuntime); // time limit
    start = clock();
    if (status = CPXmipopt(env, lp)) {
        printf("Unable to optimize the SOCP reformulation\n");
    }
    end = clock();

    //Sets the maximum number of nodes solved before the algorithm terminates without reaching optimality. When this parameter is set to 0 (zero), CPLEX completes processing at the root; that is, it creates cuts and applies heuristics at the root. When this parameter is set to 1 (one), it allows branching from the root; that is, nodes are created but not solved.


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
        printf("MIP_ObjVal: %f\n", objval);
        printf("MIP_Gap: %f\n", gap);
        printf("MIP_Nodes: %d\n", nodecount);
        printf("MIP_Runtime: %f\n", NodesRuntime);

        //写入到文件
        fprintf(output, "Objval:%.6f;Gap:%.4f;TotalTime:%.4f;Nodes:%d;Status:%d;\n", objval, gap, RootRuntime + NodesRuntime, nodecount, status);
    }

    /* Free up the problem as allocated by CPXcreateprob, if necessary */
    if (lp != NULL) {
        status = CPXfreeprob(env, &lp);
        if (status) {
            fprintf(stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
    }

    /* Free up the CPLEX environment, if necessary */
    if (env != NULL) {
        status = CPXcloseCPLEX(&env);

        /* Note that CPXcloseCPLEX produces no output,
           so the only way to see the cause of the error is to use
           CPXgeterrorstring.  For other CPLEX routines, the errors will
           be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */
        if (status) {
            fprintf(stderr, "Could not close CPLEX environment.\n");
            CPXgeterrorstring(env, status, errmsg);
            fprintf(stderr, "%s", errmsg);
        }
    }   
}

/********************************************************************************************************/
static int CPXPUBLIC PC_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle)
/********************************************************************************************************/
{
    int status = 0;
    // usercut参数
    int purgeable = CPX_USECUT_FILTER;//决定约束的保留策略：CPX_USECUT_FORCE：一直保留（最严格）CPX_USECUT_PURGE：认为约束无效可移除（较灵活）CPX_USECUT_FILTER：应用其过滤过程，并且有可能不会将该剪枝约束添加到松弛问题中，部分使用（最灵活）
    int local = 0;//局部有效（值 = 1），全局有效（值 = 0）。
    char sense = 'G';
    double rhs = 0.0;
    int cutbeg = 0;
    int cutind[] = {0, 0, 0};
    double cutval[] = {0.0, 0.0, 1.0};

    // frac: the value of x/y
    double frac = 0.0;
    double lhs = 0.0;
    int numcols = N + 2 * N * M;
    double* solBuffer = create_double_vector(numcols);

    //int nodeNum = -1;
    //CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODECOUNT, &nodeNum);
    //if (nodeNum == 0 && contextid == CPX_CALLBACKCONTEXT_RELAXATION) {
    if (contextid == CPX_CALLBACKCONTEXT_RELAXATION) {
        //获取 LP 松弛解 
        status =  CPXcallbackgetrelaxationpoint(context, solBuffer, 0, numcols - 1, NULL);
        if (status) {
            fprintf(stderr, "Failed to get relaxation solution.\n");
            free(solBuffer);
            return status;
        }
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                if (solBuffer[i] == 0.00)//去除0/0
                    frac = 0.0;
                else
                    frac = solBuffer[N + varindex(i, j)] / solBuffer[i];
                lhs = solBuffer[varindex(i, j) + N * M + N] - quad_cost[i][j] * solBuffer[N + varindex(i, j)] * frac;
                // 添加违反约束
                if (lhs < -0.0001) {
                    cutind[0] = i;
                    cutval[0] = quad_cost[i][j] * frac * frac;

                    cutind[1] = N + varindex(i, j);
                    cutval[1] = -2 * quad_cost[i][j] * frac;

                    cutind[2] = varindex(i, j) + N * M + N;
                    //添加 User Cut
                    status = CPXcallbackaddusercuts(context, 1, 3, &rhs, &sense, &cutbeg, cutind, cutval, &purgeable, &local);
                    if (status) {
                        fprintf(stderr, "Failed to add user cut.\n");
                    }
                }
            }
        }        
    }

    if (contextid == CPX_CALLBACKCONTEXT_CANDIDATE) {
        //获取整数解
        CPXcallbackgetcandidatepoint(context, solBuffer, 0, numcols - 1, NULL);
        if (status) {
            fprintf(stderr, "Failed to get candidate solution.\n");
            free(solBuffer);
            return status;
        }

        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                if (solBuffer[i] == 0.00)//去除0/0
                    frac = 0.0;
                else
                    frac = solBuffer[N + varindex(i, j)] / solBuffer[i];
                lhs = solBuffer[varindex(i, j) + N * M + N] - quad_cost[i][j] * solBuffer[N + varindex(i, j)] * frac;
                // 添加违反约束
                if (lhs < -0.0001) {
                    cutind[0] = i;
                    cutval[0] = quad_cost[i][j] * frac * frac;

                    cutind[1] = N + varindex(i, j);
                    cutval[1] = -2 * quad_cost[i][j] * frac;

                    cutind[2] = varindex(i, j) + N * M + N;
                    //添加 Lazy Constraint
                    status = CPXcallbackrejectcandidate(context, 1, 3, &rhs, &sense, &cutbeg, cutind, cutval);
                    if (status) {
                        fprintf(stderr, "Failed to add lazy constraint.\n");
                    }
                }
            }
        }
        
    }
    free(solBuffer);
    return status;
}