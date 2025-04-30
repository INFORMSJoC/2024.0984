#include "def.h"

void AP2R() {
    clock_t start, end;
    int status, nodecount;
    double NodesRuntime, objval, gap;
    UpperBound = CPX_INFBOUND;

    //建立求解环境
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
    strcpy(probname, "AP2R");
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
    double obj_sum = 0.0;
    // rational power functions
    if (P == 2.0) {
        int numvars = N * 3;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 10);

        for (int i = 0; i < N; i++) {
            if (fixed_cost[i] > quad_cost[i])
                obj[i] = fixed_cost[i] + quad_cost[i];
            else
                obj[i] = 2 * fixed_cost[i];

            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';

            sprintf(varnames[i], "y_(%d)", i);

            if (fixed_cost[i] > quad_cost[i])
                obj[i + N] = 0;
            else
                obj[i + N] = 2 * sqrt(fixed_cost[i] * quad_cost[i]);
            lb[i + N] = -CPX_INFBOUND;
            ub[i + N] = CPX_INFBOUND;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "z_(%d)", i);

            obj[i + 2 * N] = 0;
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "x_(%d)", i);
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);

        /* add linear constraints */
        int numconstrs = 1 + 2 * N + N;
        int numnz = N + 4 * N + 3 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 20);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        //single equal constraint
        cbeg[0] = 0;
        for (int i = 0; i < N; i++) {
            cind[i] = i + 2 * N;
            cval[i] = 1;
        }
        rhs[0] = 1;
        sense[0] = 'E';
        sprintf(constrnames[0], "con");

        //linked constraints
        //(0 - sqrt(fixed_cost[i]/ quad_cost[i]))*y - z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + i] = 1 * N + 2 * i;
            cind[1 * N + 2 * i] = i;
            cind[1 * N + 2 * i + 1] = i + N;
            if (fixed_cost[i] > quad_cost[i]) {
                cval[1 * N + 2 * i] = 0;
                cval[1 * N + 2 * i + 1] = 0;
            }
            else {
                cval[1 * N + 2 * i] = 0 - sqrt(fixed_cost[i] / quad_cost[i]);
                cval[1 * N + 2 * i + 1] = -1.0;
            }

            rhs[1 + i] = 0;
            sense[1 + i] = 'L';
            sprintf(constrnames[1 + i], "lb(%d)", i);
        }

        //- (1 - sqrt(fixed_cost[i] / quad_cost[i]))*y + z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + N + i] = 3 * N + 2 * i;
            cind[3 * N + 2 * i] = i;
            cind[3 * N + 2 * i + 1] = i + N;

            if (fixed_cost[i] > quad_cost[i]) {
                cval[3 * N + 2 * i] = 0;
                cval[3 * N + 2 * i + 1] = 0;
            }
            else {
                cval[3 * N + 2 * i] = -(1 - sqrt(fixed_cost[i] / quad_cost[i]));
                cval[3 * N + 2 * i + 1] = 1.0;
            }

            rhs[1 + N + i] = 0;
            sense[1 + N + i] = 'L';
            sprintf(constrnames[1 + N + i], "ub(%d)", i);
        }

        // x = x_int * y + z or x = y
        for (int i = 0; i < N; i++) {
            cbeg[1 + 2 * N + i] = 5 * N + 3 * i;
            if (fixed_cost[i] > quad_cost[i]) {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = 1;

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 0;
            }
            else {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = sqrt(fixed_cost[i] / quad_cost[i]);

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 1.0;
            }
            cind[5 * N + 3 * i + 2] = i + 2 * N;
            cval[5 * N + 3 * i + 2] = -1.0;
            rhs[1 + 2 * N + i] = 0;
            sense[1 + 2 * N + i] = 'E';
            sprintf(constrnames[1 + 2 * N + i], "equal(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free(cbeg);
        free(cind);
        free(cval);
        free_stringarray(constrnames, numconstrs);

        // 添加二次可分离目标函数
        double* qsepvec = create_double_vector(3 * N);
        for (int i = 0; i < N; i++) {
            qsepvec[i] = 0.0;
            qsepvec[i + 2 * N] = 0.0;
            if (fixed_cost[i] > quad_cost[i])
                qsepvec[i + N] = 0.0;
            else
                qsepvec[i + N] = 2 * quad_cost[i];
        }

        status = CPXcopyqpsep(env, lp, qsepvec);
        if (status)
            fprintf(stderr, "CPXcopyqpsep failed.\n");
        free(qsepvec);
    }

    if (P == 1.5) {
        int numvars = N * 6;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 10);

        for (int i = 0; i < N; i++) {
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i] = fixed_cost[i] + quad_cost[i];
            else {
                obj[i] = P * fixed_cost[i] / (P - 1);
                obj_sum += fixed_cost[i] - P * fixed_cost[i] / (P - 1);
            }

            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);


            obj[i + N] = 0;
            lb[i + N] = -CPX_INFBOUND;
            ub[i + N] = CPX_INFBOUND;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "z_(%d)", i);

            obj[i + 2 * N] = 0;
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "x_(%d)", i);

            //二阶锥约束的上镜图变量s
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i + 3 * N] = 0;
            else
                obj[i + 3 * N] = quad_cost[i];
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = CPX_INFBOUND;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "s_(%d)", i);

            //引入二阶锥约束的辅助变量t = x_int + z
            obj[i + 4 * N] = 0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = CPX_INFBOUND;
            vtype[i + 4 * N] = 'C';
            sprintf(varnames[i + 4 * N], "t_(%d)", i);

            //引入二阶锥约束的辅助变量w
            obj[i + 5 * N] = 0;
            lb[i + 5 * N] = 0;
            ub[i + 5 * N] = CPX_INFBOUND;
            vtype[i + 5 * N] = 'C';
            sprintf(varnames[i + 5 * N], "w_(%d)", i);
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);

        /* add linear constraints */
        int numconstrs = 1 + 2 * N + N + N;
        int numnz = N + 4 * N + 3 * N + 2 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 20);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        //single equal constraint
        cbeg[0] = 0;
        for (int i = 0; i < N; i++) {
            cind[i] = i + 2 * N;
            cval[i] = 1;
        }
        rhs[0] = 1;
        sense[0] = 'E';
        sprintf(constrnames[0], "con");

        //linked constraints
        //(0 - x_int)*y - z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + i] = 1 * N + 2 * i;
            cind[1 * N + 2 * i] = i;
            cind[1 * N + 2 * i + 1] = i + N;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[1 * N + 2 * i] = 0;
                cval[1 * N + 2 * i + 1] = 0;
            }
            else {
                cval[1 * N + 2 * i] = 0 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
                cval[1 * N + 2 * i + 1] = -1.0;
            }

            rhs[1 + i] = 0;
            sense[1 + i] = 'L';
            sprintf(constrnames[1 + i], "lb(%d)", i);
        }

        //- (1 - x_int)*y + z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + N + i] = 3 * N + 2 * i;
            cind[3 * N + 2 * i] = i;
            cind[3 * N + 2 * i + 1] = i + N;

            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[3 * N + 2 * i] = 0;
                cval[3 * N + 2 * i + 1] = 0;
            }
            else {
                cval[3 * N + 2 * i] = -(1 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P));
                cval[3 * N + 2 * i + 1] = 1.0;
            }

            rhs[1 + N + i] = 0;
            sense[1 + N + i] = 'L';
            sprintf(constrnames[1 + N + i], "ub(%d)", i);
        }

        // x = x_int * y + z or x = y
        for (int i = 0; i < N; i++) {
            cbeg[1 + 2 * N + i] = 5 * N + 3 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = 1;

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 0;
            }
            else {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 1.0;
            }
            cind[5 * N + 3 * i + 2] = i + 2 * N;
            cval[5 * N + 3 * i + 2] = -1.0;

            rhs[1 + 2 * N + i] = 0;
            sense[1 + 2 * N + i] = 'E';
            sprintf(constrnames[1 + 2 * N + i], "equal(%d)", i);
        }

        // t = x_int  + z 
        for (int i = 0; i < N; i++) {
            cbeg[1 + 3 * N + i] = 8 * N + 2 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[8 * N + 2 * i] = i + N;
                cval[8 * N + 2 * i] = 0;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 0;

                rhs[1 + 3 * N + i] = 0;
            }
            else {
                cind[8 * N + 2 * i] = i + N;;
                cval[8 * N + 2 * i] = -1;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 1.0;

                rhs[1 + 3 * N + i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
            }

            sense[1 + 3 * N + i] = 'E';
            sprintf(constrnames[1 + 3 * N + i], "equal2(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free(cbeg);
        free(cind);
        free(cval);
        free_stringarray(constrnames, numconstrs);

        //添加二阶锥约束
        char qsense = 'G';
        double qrhs = 0.0;
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];
        //t^2 <= ws
        for (int i = 0; i < N; i++) {
            qrow[0] = 3 * N + i;
            qcol[0] = 5 * N + i;
            qrow[1] = 4 * N + i;
            qcol[1] = 4 * N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, 2, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }

        //w^2 <= t
        //status = CPXaddqconstr(env, lp, linnzcnt, quadnzcnt, rhsval, sense, linind, linval, quadrow, quadcol, quadval, NULL);
        int linind[] = { 0 };
        double linval[] = { 1.0 };
        int qrow2[] = { 0 };
        int qcol2[] = { 0 };
        double qval2[] = { -1 };
        for (int i = 0; i < N; i++) {
            linind[0] = 4 * N + i;

            qrow2[0] = 5 * N + i;
            qcol2[0] = 5 * N + i;

            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 1, 1, qrhs, qsense, linind, linval, qrow2, qcol2, qval2, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 2.5) {
        int numvars = N * 8;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 10);

        for (int i = 0; i < N; i++) {
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i] = fixed_cost[i] + quad_cost[i];
            else {
                obj[i] = P * fixed_cost[i] / (P - 1);
                obj_sum += fixed_cost[i] - P * fixed_cost[i] / (P - 1);
            }

            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);


            obj[i + N] = 0;
            lb[i + N] = -CPX_INFBOUND;
            ub[i + N] = CPX_INFBOUND;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "z_(%d)", i);

            obj[i + 2 * N] = 0;
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "x_(%d)", i);

            //二阶锥约束的上镜图变量s
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i + 3 * N] = 0;
            else
                obj[i + 3 * N] = quad_cost[i];
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = CPX_INFBOUND;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "s_(%d)", i);

            //引入二阶锥约束的辅助变量t = x_int + z
            obj[i + 4 * N] = 0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = CPX_INFBOUND;
            vtype[i + 4 * N] = 'C';
            sprintf(varnames[i + 4 * N], "t_(%d)", i);
            //引入二阶锥约束的辅助变量w
            obj[i + 5 * N] = 0;
            lb[i + 5 * N] = 0;
            ub[i + 5 * N] = CPX_INFBOUND;
            vtype[i + 5 * N] = 'C';
            sprintf(varnames[i + 5 * N], "w_(%d)", i);
            //引入二阶锥约束的辅助变量u
            obj[i + 6 * N] = 0;
            lb[i + 6 * N] = 0;
            ub[i + 6 * N] = CPX_INFBOUND;
            vtype[i + 6 * N] = 'C';
            sprintf(varnames[i + 6 * N], "u_(%d)", i);
            //引入二阶锥约束的辅助变量v
            obj[i + 7 * N] = 0;
            lb[i + 7 * N] = 0;
            ub[i + 7 * N] = CPX_INFBOUND;
            vtype[i + 7 * N] = 'C';
            sprintf(varnames[i + 7 * N], "v_(%d)", i);
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);

        /* add linear constraints */
        int numconstrs = 1 + 2 * N + N + N;
        int numnz = N + 4 * N + 3 * N + 2 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 20);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        //single equal constraint
        cbeg[0] = 0;
        for (int i = 0; i < N; i++) {
            cind[i] = i + 2 * N;
            cval[i] = 1;
        }
        rhs[0] = 1;
        sense[0] = 'E';
        sprintf(constrnames[0], "con");

        //linked constraints
        //(0 - x_int)*y - z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + i] = 1 * N + 2 * i;
            cind[1 * N + 2 * i] = i;
            cind[1 * N + 2 * i + 1] = i + N;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[1 * N + 2 * i] = 0;
                cval[1 * N + 2 * i + 1] = 0;
            }
            else {
                cval[1 * N + 2 * i] = 0 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
                cval[1 * N + 2 * i + 1] = -1.0;
            }

            rhs[1 + i] = 0;
            sense[1 + i] = 'L';
            sprintf(constrnames[1 + i], "lb(%d)", i);
        }

        //- (1 - x_int)*y + z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + N + i] = 3 * N + 2 * i;
            cind[3 * N + 2 * i] = i;
            cind[3 * N + 2 * i + 1] = i + N;

            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[3 * N + 2 * i] = 0;
                cval[3 * N + 2 * i + 1] = 0;
            }
            else {
                cval[3 * N + 2 * i] = -(1 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P));
                cval[3 * N + 2 * i + 1] = 1.0;
            }

            rhs[1 + N + i] = 0;
            sense[1 + N + i] = 'L';
            sprintf(constrnames[1 + N + i], "ub(%d)", i);
        }

        // x = x_int * y + z or x = y
        for (int i = 0; i < N; i++) {
            cbeg[1 + 2 * N + i] = 5 * N + 3 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = 1;

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 0;
            }
            else {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 1.0;
            }
            cind[5 * N + 3 * i + 2] = i + 2 * N;
            cval[5 * N + 3 * i + 2] = -1.0;

            rhs[1 + 2 * N + i] = 0;
            sense[1 + 2 * N + i] = 'E';
            sprintf(constrnames[1 + 2 * N + i], "equal(%d)", i);
        }

        // t = x_int  + z 
        for (int i = 0; i < N; i++) {
            cbeg[1 + 3 * N + i] = 8 * N + 2 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[8 * N + 2 * i] = i + N;
                cval[8 * N + 2 * i] = 0;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 0;

                rhs[1 + 3 * N + i] = 0;
            }
            else {
                cind[8 * N + 2 * i] = i + N;;
                cval[8 * N + 2 * i] = -1;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 1.0;

                rhs[1 + 3 * N + i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
            }

            sense[1 + 3 * N + i] = 'E';
            sprintf(constrnames[1 + 3 * N + i], "equal2(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free(cbeg);
        free(cind);
        free(cval);
        free_stringarray(constrnames, numconstrs);

        //添加二阶锥约束
        char qsense = 'G';
        double qrhs = 0.0;
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];
        //t^2 <= uv
        for (int i = 0; i < N; i++) {
            qrow[0] = 6 * N + i;
            qcol[0] = 7 * N + i;
            qrow[1] = 4 * N + i;
            qcol[1] = 4 * N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, 2, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
        //u^2 <= tw
        for (int i = 0; i < N; i++) {
            qrow[0] = 4 * N + i;
            qcol[0] = 5 * N + i;
            qrow[1] = 6 * N + i;
            qcol[1] = 6 * N + i;
            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 0, 2, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }

        //w^2 <= t
        //status = CPXaddqconstr(env, lp, linnzcnt, quadnzcnt, rhsval, sense, linind, linval, quadrow, quadcol, quadval, NULL);
        int linind[] = { 0 };
        double linval[] = { 1.0 };
        int qrow2[] = { 0 };
        int qcol2[] = { 0 };
        double qval2[] = { -1 };
        for (int i = 0; i < N; i++) {
            linind[0] = 4 * N + i;

            qrow2[0] = 5 * N + i;
            qcol2[0] = 5 * N + i;

            sprintf(qcname, "qcon3(%d)", i);
            status = CPXaddqconstr(env, lp, 1, 1, qrhs, qsense, linind, linval, qrow2, qcol2, qval2, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
        //v^2 <= s
        for (int i = 0; i < N; i++) {
            linind[0] = 3 * N + i;

            qrow2[0] = 7 * N + i;
            qcol2[0] = 7 * N + i;

            sprintf(qcname, "qcon4(%d)", i);
            status = CPXaddqconstr(env, lp, 1, 1, qrhs, qsense, linind, linval, qrow2, qcol2, qval2, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 3.0) {
        int numvars = N * 6;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 10);

        for (int i = 0; i < N; i++) {
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i] = fixed_cost[i] + quad_cost[i];
            else {
                obj[i] = P * fixed_cost[i] / (P - 1);
                obj_sum += fixed_cost[i] - P * fixed_cost[i] / (P - 1);
            }

            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);


            obj[i + N] = 0;
            lb[i + N] = -CPX_INFBOUND;
            ub[i + N] = CPX_INFBOUND;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "z_(%d)", i);

            obj[i + 2 * N] = 0;
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "x_(%d)", i);

            //二阶锥约束的上镜图变量s
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1)
                obj[i + 3 * N] = 0;
            else
                obj[i + 3 * N] = quad_cost[i];
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = CPX_INFBOUND;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "s_(%d)", i);

            //引入二阶锥约束的辅助变量t = x_int + z
            obj[i + 4 * N] = 0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = CPX_INFBOUND;
            vtype[i + 4 * N] = 'C';
            sprintf(varnames[i + 4 * N], "t_(%d)", i);
            //引入二阶锥约束的辅助变量w
            obj[i + 5 * N] = 0;
            lb[i + 5 * N] = 0;
            ub[i + 5 * N] = CPX_INFBOUND;
            vtype[i + 5 * N] = 'C';
            sprintf(varnames[i + 5 * N], "w_(%d)", i);
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);

        /* add linear constraints */
        int numconstrs = 1 + 2 * N + N + N;
        int numnz = N + 4 * N + 3 * N + 2 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 20);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        //single equal constraint
        cbeg[0] = 0;
        for (int i = 0; i < N; i++) {
            cind[i] = i + 2 * N;
            cval[i] = 1;
        }
        rhs[0] = 1;
        sense[0] = 'E';
        sprintf(constrnames[0], "con");

        //linked constraints
        //(0 - x_int)*y - z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + i] = 1 * N + 2 * i;
            cind[1 * N + 2 * i] = i;
            cind[1 * N + 2 * i + 1] = i + N;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[1 * N + 2 * i] = 0;
                cval[1 * N + 2 * i + 1] = 0;
            }
            else {
                cval[1 * N + 2 * i] = 0 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
                cval[1 * N + 2 * i + 1] = -1.0;
            }

            rhs[1 + i] = 0;
            sense[1 + i] = 'L';
            sprintf(constrnames[1 + i], "lb(%d)", i);
        }

        //- (1 - x_int)*y + z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + N + i] = 3 * N + 2 * i;
            cind[3 * N + 2 * i] = i;
            cind[3 * N + 2 * i + 1] = i + N;

            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cval[3 * N + 2 * i] = 0;
                cval[3 * N + 2 * i + 1] = 0;
            }
            else {
                cval[3 * N + 2 * i] = -(1 - pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P));
                cval[3 * N + 2 * i + 1] = 1.0;
            }

            rhs[1 + N + i] = 0;
            sense[1 + N + i] = 'L';
            sprintf(constrnames[1 + N + i], "ub(%d)", i);
        }

        // x = x_int * y + z or x = y
        for (int i = 0; i < N; i++) {
            cbeg[1 + 2 * N + i] = 5 * N + 3 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = 1;

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 0;
            }
            else {
                cind[5 * N + 3 * i] = i;
                cval[5 * N + 3 * i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);

                cind[5 * N + 3 * i + 1] = i + N;
                cval[5 * N + 3 * i + 1] = 1.0;
            }
            cind[5 * N + 3 * i + 2] = i + 2 * N;
            cval[5 * N + 3 * i + 2] = -1.0;

            rhs[1 + 2 * N + i] = 0;
            sense[1 + 2 * N + i] = 'E';
            sprintf(constrnames[1 + 2 * N + i], "equal(%d)", i);
        }

        // t = x_int  + z 
        for (int i = 0; i < N; i++) {
            cbeg[1 + 3 * N + i] = 8 * N + 2 * i;
            if (fixed_cost[i] / (quad_cost[i] * (P - 1)) >= 1) {
                cind[8 * N + 2 * i] = i + N;
                cval[8 * N + 2 * i] = 0;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 0;

                rhs[1 + 3 * N + i] = 0;
            }
            else {
                cind[8 * N + 2 * i] = i + N;;
                cval[8 * N + 2 * i] = -1;

                cind[8 * N + 2 * i + 1] = i + 4 * N;
                cval[8 * N + 2 * i + 1] = 1.0;

                rhs[1 + 3 * N + i] = pow(fixed_cost[i] / (quad_cost[i] * (P - 1)), 1.0 / P);
            }

            sense[1 + 3 * N + i] = 'E';
            sprintf(constrnames[1 + 3 * N + i], "equal2(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free(cbeg);
        free(cind);
        free(cval);
        free_stringarray(constrnames, numconstrs);

        //添加二阶锥约束
        char qsense = 'G';
        double qrhs = 0.0;
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];
        //w^2 <= st
        for (int i = 0; i < N; i++) {
            qrow[0] = 3 * N + i;
            qcol[0] = 4 * N + i;
            qrow[1] = 5 * N + i;
            qcol[1] = 5 * N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, 2, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }

        //t^2 <= w
        //status = CPXaddqconstr(env, lp, linnzcnt, quadnzcnt, rhsval, sense, linind, linval, quadrow, quadcol, quadval, NULL);
        int linind[] = { 0 };
        double linval[] = { 1.0 };
        int qrow2[] = { 0 };
        int qcol2[] = { 0 };
        double qval2[] = { -1 };
        for (int i = 0; i < N; i++) {
            linind[0] = 5 * N + i;

            qrow2[0] = 4 * N + i;
            qcol2[0] = 4 * N + i;

            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 1, 1, qrhs, qsense, linind, linval, qrow2, qcol2, qval2, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 0) {//Kleinrock average delay function
        int numvars = N * 7;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 10);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i] + sqrt(quad_cost[i] * fixed_cost[i]);
            obj_sum -= sqrt(quad_cost[i] * fixed_cost[i]);
            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);


            obj[i + N] = 0;
            lb[i + N] = -CPX_INFBOUND;
            ub[i + N] = CPX_INFBOUND;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "z_(%d)", i);

            obj[i + 2 * N] = 0;
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "x_(%d)", i);

            //二阶锥约束的上镜图变量s
            obj[i + 3 * N] = quad_cost[i];
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = CPX_INFBOUND;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "s_(%d)", i);

            //引入二阶锥约束的辅助变量t = x_int + z
            obj[i + 4 * N] = 0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = CPX_INFBOUND;
            vtype[i + 4 * N] = 'C';
            sprintf(varnames[i + 4 * N], "t_(%d)", i);

            //引入二阶锥约束的辅助变量u
            obj[i + 5 * N] = 0;
            lb[i + 5 * N] = 0;
            ub[i + 5 * N] = CPX_INFBOUND;
            vtype[i + 5 * N] = 'C';
            sprintf(varnames[i + 5 * N], "u_(%d)", i);
            //引入二阶锥约束的辅助变量v
            obj[i + 6 * N] = 0;
            lb[i + 6 * N] = 0;
            ub[i + 6 * N] = CPX_INFBOUND;
            vtype[i + 6 * N] = 'C';
            sprintf(varnames[i + 6 * N], "v_(%d)", i);
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);

        /* add linear constraints */
        int numconstrs = 1 + 2 * N + N + N + N + N;
        int numnz = N + 4 * N + 3 * N + 2 * N + 3 * N + 2 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 20);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        //single equal constraint
        cbeg[0] = 0;
        for (int i = 0; i < N; i++) {
            cind[i] = i + 2 * N;
            cval[i] = 1;
        }
        rhs[0] = 1;
        sense[0] = 'E';
        sprintf(constrnames[0], "con");

        //linked constraints
        //(0 - x_int)*y - z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + i] = 1 * N + 2 * i;
            cind[1 * N + 2 * i] = i;
            cind[1 * N + 2 * i + 1] = i + N;
            cval[1 * N + 2 * i] = 0 - 1 / (1 + sqrt(quad_cost[i] / fixed_cost[i]));
            cval[1 * N + 2 * i + 1] = -1.0;


            rhs[1 + i] = 0;
            sense[1 + i] = 'L';
            sprintf(constrnames[1 + i], "lb(%d)", i);
        }

        //- (1 - x_int)*y + z <= 0
        for (int i = 0; i < N; i++) {
            cbeg[1 + N + i] = 3 * N + 2 * i;
            cind[3 * N + 2 * i] = i;
            cind[3 * N + 2 * i + 1] = i + N;
            cval[3 * N + 2 * i] = -(1 - 1 / (1 + sqrt(quad_cost[i] / fixed_cost[i])));
            cval[3 * N + 2 * i + 1] = 1.0;


            rhs[1 + N + i] = 0;
            sense[1 + N + i] = 'L';
            sprintf(constrnames[1 + N + i], "ub(%d)", i);
        }

        // x = x_int * y + z 
        for (int i = 0; i < N; i++) {
            cbeg[1 + 2 * N + i] = 5 * N + 3 * i;
            cind[5 * N + 3 * i] = i;
            cval[5 * N + 3 * i] = 1 / (1 + sqrt(quad_cost[i] / fixed_cost[i]));

            cind[5 * N + 3 * i + 1] = i + N;
            cval[5 * N + 3 * i + 1] = 1.0;

            cind[5 * N + 3 * i + 2] = i + 2 * N;
            cval[5 * N + 3 * i + 2] = -1.0;

            rhs[1 + 2 * N + i] = 0;
            sense[1 + 2 * N + i] = 'E';
            sprintf(constrnames[1 + 2 * N + i], "equal(%d)", i);
        }

        // t = x_int  + z 
        for (int i = 0; i < N; i++) {
            cbeg[1 + 3 * N + i] = 8 * N + 2 * i;

            cind[8 * N + 2 * i] = i + N;;
            cval[8 * N + 2 * i] = -1;

            cind[8 * N + 2 * i + 1] = i + 4 * N;
            cval[8 * N + 2 * i + 1] = 1.0;

            rhs[1 + 3 * N + i] = 1 / (1 + sqrt(quad_cost[i] / fixed_cost[i]));
            sense[1 + 3 * N + i] = 'E';
            sprintf(constrnames[1 + 3 * N + i], "equal2(%d)", i);
        }

        //u = s - t
        for (int i = 0; i < N; i++) {
            cbeg[1 + 4 * N + i] = 10 * N + 3 * i;
            cind[10 * N + 3 * i] = i + 3 * N;
            cval[10 * N + 3 * i] = 1;

            cind[10 * N + 3 * i + 1] = i + 5 * N;
            cval[10 * N + 3 * i + 1] = -1.0;

            cind[10 * N + 3 * i + 2] = i + 4 * N;
            cval[10 * N + 3 * i + 2] = -1.0;

            rhs[1 + 4 * N + i] = 0;
            sense[1 + 4 * N + i] = 'E';
            sprintf(constrnames[1 + 4 * N + i], "equa3(%d)", i);
        }

        //v = 1 - t
        for (int i = 0; i < N; i++) {
            cbeg[1 + 5 * N + i] = 13 * N + 2 * i;

            cind[13 * N + 2 * i] = i + 6 * N;;
            cval[13 * N + 2 * i] = 1;

            cind[13 * N + 2 * i + 1] = i + 4 * N;
            cval[13 * N + 2 * i + 1] = 1.0;

            rhs[1 + 5 * N + i] = 1;
            sense[1 + 5 * N + i] = 'E';
            sprintf(constrnames[1 + 5 * N + i], "equal4(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free(cbeg);
        free(cind);
        free(cval);
        free_stringarray(constrnames, numconstrs);

        //添加二阶锥约束
        char qsense = 'G';
        double qrhs = 0.0;
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];
        //t^2 <= uv
        for (int i = 0; i < N; i++) {
            qrow[0] = 5 * N + i;
            qcol[0] = 6 * N + i;
            qrow[1] = 4 * N + i;
            qcol[1] = 4 * N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, 2, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    // 设置求解参数 单线程 1000s
    if (P != 2.0) {
        CPXsetintparam(env, CPXPARAM_MIP_Strategy_MIQCPStrat, 1);// tell CPLEX to solve a QCP relaxation of the model at each node
    }
    CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
    CPXsetintparam(env, CPX_PARAM_THREADS, 1); // Nber of threads to use
    CPXsetdblparam(env, CPX_PARAM_EPINT, 0);
    CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9);

    // 设置目标函数的常数偏移量
    CPXchgobjoffset(env, lp, obj_sum);

    //检查模型
    //status = CPXwriteprob(env, lp, "AP2R.lp", NULL);
    //if (status)
    //    fprintf(stderr, "Failed to write LP to disk.\n");

    //求解根节点松弛解
    CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 1);
    start = clock();
    if (status = CPXmipopt(env, lp)) {
        printf("Unable to optimize the AP2R reformulation\n");
    }
    end = clock();
    RootRuntime = ((double)end - (double)start) / CLOCKS_PER_SEC;
    CPXgetmipobjval(env, lp, &UpperBound);//获取上界
    CPXgetmiprelgap(env, lp, &gap);//获取gap
    CPXgetbestobjval(env, lp, &LowerBound);//获取下界
    //根节点结果写入到文件
    fprintf(output, "RootUpperBound:%.6f;RootLowerBound:%.6f;RootGap:%.4f;RootRuntime:%.4f;\n", UpperBound, LowerBound, gap, RootRuntime);

    //开始分支节点求解
    CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 999999999);
    CPXsetdblparam(env, CPX_PARAM_TILIM, 1000- RootRuntime); // time limit
    start = clock();
    if (status = CPXmipopt(env, lp)) {
        printf("Unable to optimize the AP2R reformulation\n");
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
        printf("AP2R Problem infeasible!\n");
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
        //printf("obj_sum: %f\n", obj_sum);
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