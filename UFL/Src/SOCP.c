#include "def.h"

void SOCP() {
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

   
    if (P == 2.0) {
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

                obj[varindex(i, j) + N * M + N] = quad_cost[i][j];
                lb[varindex(i, j) + N * M + N] = 0;
                ub[varindex(i, j) + N * M + N] = 1;
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

        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qindrow[] = { 0, 0 };
        int qindcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[16];
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qindrow[0] = i;
                qindcol[0] = N + N * M + varindex(i, j);
                qindrow[1] = N + varindex(i, j);
                qindcol[1] = N + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qindrow, qindcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }
    }

    if (P == 1.5) {
        int numvars = N + 3 * N * M;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 100);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);

            //二维变量定义
            for (int j = 0; j < M; j++) {
                obj[varindex(i, j) + N] = 0;
                lb[varindex(i, j) + N] = 0;
                ub[varindex(i, j) + N] = 1;
                vtype[varindex(i, j) + N] = 'C';
                sprintf(varnames[varindex(i, j) + N], "x_(%d,%d)", i, j);

                obj[varindex(i, j) + N * M + N] = quad_cost[i][j];
                lb[varindex(i, j) + N * M + N] = 0;
                ub[varindex(i, j) + N * M + N] = 1;
                vtype[varindex(i, j) + N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + N * M + N], "z_(%d,%d)", i, j);

                obj[varindex(i, j) + 2 * N * M + N] = 0;
                lb[varindex(i, j) + 2 * N * M + N] = 0;
                ub[varindex(i, j) + 2 * N * M + N] = 1;
                vtype[varindex(i, j) + 2 * N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + 2 * N * M + N], "w_(%d,%d)", i, j);
            }
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);


        // 添加线性约束
        int numconstrs = N * M + M;
        int numnz = 2 * N * M + N * M;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 100);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                // 所有非零元素排成一行 因为每行2个元素，每行的起始索引为 0 2 4 ...
                cbeg[j * N + i] = 2 * j * N + 2 * i;
                //每行有两个元素 1 -1，一个在i列，一个在i+N列
                cind[2 * j * N + 2 * i] = i;
                cval[2 * j * N + 2 * i] = 1.0;
                cind[2 * j * N + 2 * i + 1] = N + varindex(i, j);
                cval[2 * j * N + 2 * i + 1] = -1.0;

                rhs[j * N + i] = 0;
                sense[j * N + i] = 'G';
                sprintf(constrnames[j * N + i], "link(%d,%d)", i, j);
            }
        }

        for (int j = 0; j < M; j++) {
            cbeg[N * M + j] = 2 * N * M + (j * N);
            for (int i = 0; i < N; i++) {
                //最后一行， 每一列都有一个1     
                cind[2 * N * M + (j * N) + i] = N + varindex(i, j);
                cval[2 * N * M + (j * N) + i] = 1.0;
            }
            rhs[N * M + j] = 1;
            sense[N * M + j] = 'E';
            sprintf(constrnames[N * M + j], "con(%d)", j);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        // 添加二次约束
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[100];
        //添加x^2<= wz
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + 2 * N * M + varindex(i, j);
                qcol[0] = N + N * M + varindex(i, j);
                qrow[1] = N + varindex(i, j);
                qcol[1] = N + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }

        //添加w^2<= xy
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + varindex(i, j);
                qcol[0] = i;
                qrow[1] = N + 2 * N * M + varindex(i, j);
                qcol[1] = N + 2 * N * M + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }
    }

    if (P == 2.5) {
        int numvars = N + 5 * N * M;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 100);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1; 
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);

            //二维变量定义
            for (int j = 0; j < M; j++) {
                obj[varindex(i, j) + N] = 0;
                lb[varindex(i, j) + N] = 0;
                ub[varindex(i, j) + N] = 1;
                vtype[varindex(i, j) + N] = 'C';
                sprintf(varnames[varindex(i, j) + N], "x_(%d,%d)", i, j);

                obj[varindex(i, j) + N * M + N] = quad_cost[i][j];
                lb[varindex(i, j) + N * M + N] = 0;
                ub[varindex(i, j) + N * M + N] = 1;
                vtype[varindex(i, j) + N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + N * M + N], "z_(%d,%d)", i, j);

                obj[varindex(i, j) + 2 * N * M + N] = 0;
                lb[varindex(i, j) + 2 * N * M + N] = 0;
                ub[varindex(i, j) + 2 * N * M + N] = 1;
                vtype[varindex(i, j) + 2 * N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + 2 * N * M + N], "w1_(%d,%d)", i, j);

                obj[varindex(i, j) + 3 * N * M + N] = 0;
                lb[varindex(i, j) + 3 * N * M + N] = 0;
                ub[varindex(i, j) + 3 * N * M + N] = 1;
                vtype[varindex(i, j) + 3 * N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + 3 * N * M + N], "w2_(%d,%d)", i, j);

                obj[varindex(i, j) + 4 * N * M + N] = 0;
                lb[varindex(i, j) + 4 * N * M + N] = 0;
                ub[varindex(i, j) + 4 * N * M + N] = 1;
                vtype[varindex(i, j) + 4 * N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + 4 * N * M + N], "w3_(%d,%d)", i, j);
            }
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);


        // 添加线性约束
        int numconstrs = N * M + M;
        int numnz = 2 * N * M + N * M;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 100);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                // 所有非零元素排成一行 因为每行2个元素，每行的起始索引为 0 2 4 ...
                cbeg[j * N + i] = 2 * j * N + 2 * i;
                //每行有两个元素 1 -1，一个在i列，一个在i+N列
                cind[2 * j * N + 2 * i] = i;
                cval[2 * j * N + 2 * i] = 1.0;
                cind[2 * j * N + 2 * i + 1] = N + varindex(i, j);
                cval[2 * j * N + 2 * i + 1] = -1.0;

                rhs[j * N + i] = 0;
                sense[j * N + i] = 'G';
                sprintf(constrnames[j * N + i], "link(%d,%d)", i, j);
            }
        }

        for (int j = 0; j < M; j++) {
            cbeg[N * M + j] = 2 * N * M + (j * N);
            for (int i = 0; i < N; i++) {
                //最后一行， 每一列都有一个1     
                cind[2 * N * M + (j * N) + i] = N + varindex(i, j);
                cval[2 * N * M + (j * N) + i] = 1.0;
            }
            rhs[N * M + j] = 1;
            sense[N * M + j] = 'E';
            sprintf(constrnames[N * M + j], "con(%d)", j);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        // 添加二次约束
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[100];
        //添加x^2<= w2w3
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + 3 * N * M + varindex(i, j);
                qcol[0] = N + 4 * N * M + varindex(i, j);
                qrow[1] = N + varindex(i, j);
                qcol[1] = N + varindex(i, j);
                sprintf(qcname, "qcon1(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }

        //添加w3^2<= yz
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = i;
                qcol[0] = N + N * M + varindex(i, j);
                qrow[1] = N + 4 * N * M + varindex(i, j);
                qcol[1] = N + 4 * N * M + varindex(i, j);
                sprintf(qcname, "qcon2(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }

        //添加w2^2<= w1x
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + 2 * N * M + varindex(i, j);
                qcol[0] = N + varindex(i, j);
                qrow[1] = N + 3 * N * M + varindex(i, j);
                qcol[1] = N + 3 * N * M + varindex(i, j);
                sprintf(qcname, "qcon3(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }

        //添加w1^2<= xy
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = i;
                qcol[0] = N + varindex(i, j);
                qrow[1] = N + 2 * N * M + varindex(i, j);
                qcol[1] = N + 2 * N * M + varindex(i, j);
                sprintf(qcname, "qcon4(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }
    }

    if (P == 3.0) {
        int numvars = N + 3 * N * M;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 100);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);

            //二维变量定义
            for (int j = 0; j < M; j++) {
                obj[varindex(i, j) + N] = 0;
                lb[varindex(i, j) + N] = 0;
                ub[varindex(i, j) + N] = 1;
                vtype[varindex(i, j) + N] = 'C';
                sprintf(varnames[varindex(i, j) + N], "x_(%d,%d)", i, j);

                obj[varindex(i, j) + N * M + N] = quad_cost[i][j];
                lb[varindex(i, j) + N * M + N] = 0;
                ub[varindex(i, j) + N * M + N] = 1;
                vtype[varindex(i, j) + N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + N * M + N], "z_(%d,%d)", i, j);

                obj[varindex(i, j) + 2 * N * M + N] = 0;
                lb[varindex(i, j) + 2 * N * M + N] = 0;
                ub[varindex(i, j) + 2 * N * M + N] = 1;
                vtype[varindex(i, j) + 2 * N * M + N] = 'C';
                sprintf(varnames[varindex(i, j) + 2 * N * M + N], "w_(%d,%d)", i, j);
            }
        }

        status = CPXnewcols(env, lp, numvars, obj, lb, ub, vtype, varnames);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(vtype);
        free_stringarray(varnames, numvars);


        // 添加线性约束
        int numconstrs = N * M + M;
        int numnz = 2 * N * M + N * M;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 100);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);

        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                // 所有非零元素排成一行 因为每行2个元素，每行的起始索引为 0 2 4 ...
                cbeg[j * N + i] = 2 * j * N + 2 * i;
                //每行有两个元素 1 -1，一个在i列，一个在i+N列
                cind[2 * j * N + 2 * i] = i;
                cval[2 * j * N + 2 * i] = 1.0;
                cind[2 * j * N + 2 * i + 1] = N + varindex(i, j);
                cval[2 * j * N + 2 * i + 1] = -1.0;

                rhs[j * N + i] = 0;
                sense[j * N + i] = 'G';
                sprintf(constrnames[j * N + i], "link(%d,%d)", i, j);
            }
        }

        for (int j = 0; j < M; j++) {
            cbeg[N * M + j] = 2 * N * M + (j * N);
            for (int i = 0; i < N; i++) {
                //最后一行， 每一列都有一个1     
                cind[2 * N * M + (j * N) + i] = N + varindex(i, j);
                cval[2 * N * M + (j * N) + i] = 1.0;
            }
            rhs[N * M + j] = 1;
            sense[N * M + j] = 'E';
            sprintf(constrnames[N * M + j], "con(%d)", j);
        }

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        // 添加二次约束
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[100];
        //添加x^2<= wy
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + 2 * N * M + varindex(i, j);
                qcol[0] = i;
                qrow[1] = N + varindex(i, j);
                qcol[1] = N + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }

        //添加w^2<= xz
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qrow[0] = N + varindex(i, j);
                qcol[0] = N + N * M + varindex(i, j);
                qrow[1] = N + 2 * N * M + varindex(i, j);
                qcol[1] = N + 2 * N * M + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }
    }

    if (P == 0) {//Kleinrock average delay function

        //添加变量
        int numcols = N + 4 * N * M;
        double* obj = create_double_vector(numcols);
        double* lb = create_double_vector(numcols);
        double* ub = create_double_vector(numcols);
        char* xctype = create_char_vector(numcols);
        char** colname = create_stringarray(numcols, 40);

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

                obj[varindex(i, j) + N * M + N] = quad_cost[i][j];
                lb[varindex(i, j) + N * M + N] = 0;
                ub[varindex(i, j) + N * M + N] = 1;
                xctype[varindex(i, j) + N * M + N] = 'C';
                sprintf(colname[varindex(i, j) + N * M + N], "z_(%d,%d)", i, j);

                obj[varindex(i, j) + 2 * N * M + N] = 0;
                lb[varindex(i, j) + 2 * N * M + N] = 0;
                ub[varindex(i, j) + 2 * N * M + N] = 1;
                xctype[varindex(i, j) + 2 * N * M + N] = 'C';
                sprintf(colname[varindex(i, j) + 2 * N * M + N], "w1_(%d,%d)", i, j);

                obj[varindex(i, j) + 3 * N * M + N] = 0;
                lb[varindex(i, j) + 3 * N * M + N] = 0;
                ub[varindex(i, j) + 3 * N * M + N] = 1;
                xctype[varindex(i, j) + 3 * N * M + N] = 'C';
                sprintf(colname[varindex(i, j) + 3 * N * M + N], "w2_(%d,%d)", i, j);
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

        //添加线性约束
        int numrows = N * M + M + 2 * N * M;
        int numnz = 2 * N * M + N * M + 3 * 2 * N * M;
        double* rhs = create_double_vector(numrows);
        char* sense = create_char_vector(numrows);
        char** rowname = create_stringarray(numrows, 40);
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

        for (int j = 0; j < M; j++) {            
            for (int i = 0; i < N; i++) {
                //w1 = y-x
                rmatbeg[N * M + M + j * N + i] = 2 * N * M + N * M + 3 * j * N + 3 * i;

                rmatind[2 * N * M + N * M + 3 * j * N + 3 * i] = i;
                rmatval[2 * N * M + N * M + 3 * j * N + 3 * i] = 1.0;
                rmatind[2 * N * M + N * M + 3 * j * N + 3 * i + 1] = varindex(i, j) + N;
                rmatval[2 * N * M + N * M + 3 * j * N + 3 * i + 1] = -1.0;
                rmatind[2 * N * M + N * M + 3 * j * N + 3 * i + 2] = varindex(i, j) + 2 * N * M + N;
                rmatval[2 * N * M + N * M + 3 * j * N + 3 * i + 2] = -1.0;

                rhs[N * M + M + j * N + i] = 0;
                sense[N * M + M + j * N + i] = 'E';
                sprintf(rowname[N * M + M + j * N + i], "w1_c(%d,%d)", i,j);
            }
        }

        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                //w2 = z-x
                rmatbeg[2 * N * M + M + j * N + i] = 2 * N * M + 4 * N * M + 3 * j * N + 3 * i;

                rmatind[2 * N * M + 4 * N * M + 3 * j * N + 3 * i] = varindex(i, j) + N + M * N;
                rmatval[2 * N * M + 4 * N * M + 3 * j * N + 3 * i] = 1.0;
                rmatind[2 * N * M + 4 * N * M + 3 * j * N + 3 * i + 1] = varindex(i, j) + N;
                rmatval[2 * N * M + 4 * N * M + 3 * j * N + 3 * i + 1] = -1.0;
                rmatind[2 * N * M + 4 * N * M + 3 * j * N + 3 * i + 2] = varindex(i, j) + 3 * N * M + N;
                rmatval[2 * N * M + 4 * N * M + 3 * j * N + 3 * i + 2] = -1.0;

                rhs[2 * N * M + M + j * N + i] = 0;
                sense[2 * N * M + M + j * N + i] = 'E';
                sprintf(rowname[2 * N * M + M + j * N + i], "w2_c(%d,%d)", i, j);
            }
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

        //添加二阶锥约束
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qindrow[] = { 0, 0 };
        int qindcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[40];
        for (int j = 0; j < M; j++) {
            for (int i = 0; i < N; i++) {
                qindrow[0] = varindex(i, j) + 2 * N * M + N;
                qindcol[0] = varindex(i, j) + 3 * N * M + N;
                qindrow[1] = N + varindex(i, j);
                qindcol[1] = N + varindex(i, j);
                sprintf(qcname, "qcon(%d,%d)", i, j);
                status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qindrow, qindcol, qval, qcname);
                if (status)
                    fprintf(stderr, "CPXaddrows failed.\n");
            }
        }
    }

    // 设置求解参数 单线程 1000s
    CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
    CPXsetintparam(env, CPX_PARAM_THREADS, 1); // Nber of threads to use

    CPXsetdblparam(env, CPX_PARAM_EPINT, 0);
    CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9);
    if (P == 2.0)
        CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-9); // e-optimal solution (%gap): The violation tolerance of optimality	
    CPXsetintparam(env, CPXPARAM_MIP_Strategy_MIQCPStrat, 1);// tell CPLEX to solve a QCP relaxation of the model at each node
    

    //检查模型
    //status = CPXwriteprob(env, lp, "SOCP.lp", NULL);
    ////status = CPXwriteprob(env, lp, "SOCP.MPS", NULL);
    //if (status)
    //    fprintf(stderr, "Failed to write LP to disk.\n");

    //求解根节点松弛解
    CPXsetintparam(env, CPXPARAM_MIP_Limits_Nodes, 1);
    start = clock();
    if (status = CPXmipopt(env, lp)) {
        printf("Unable to optimize the SOCP reformulation\n");
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