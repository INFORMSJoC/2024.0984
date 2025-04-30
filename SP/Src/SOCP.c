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
        int numcols = N * 3;
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

            obj[i + N] = 0.0;
            lb[i + N] = 0;
            ub[i + N] = 1;
            xctype[i + N] = 'C';
            sprintf(colname[i + N], "x_(%d)", i);

            obj[i + 2 * N] = quad_cost[i];
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            xctype[i + 2 * N] = 'C';
            sprintf(colname[i + 2 * N], "z_(%d)", i);
        }

        status = CPXnewcols(env, lp, numcols, obj, lb, ub, xctype, colname);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(xctype);
        free_stringarray(colname, numcols);

        int numrows = N + 1;
        int numnz = 3 * N;
        double* rhs = create_double_vector(numrows);
        char* sense = create_char_vector(numrows);
        char** rowname = create_stringarray(numrows, 30);
        int* rmatbeg = create_int_vector(numrows);
        int* rmatind = create_int_vector(numnz);
        double* rmatval = create_double_vector(numnz);

        for (int i = 0; i < N; i++) {
            rmatbeg[i] = 2 * i;
            rmatind[2 * i] = i;
            rmatval[2 * i] = 1.0;
            rmatind[2 * i + 1] = i + N;
            rmatval[2 * i + 1] = -1.0;

            rhs[i] = 0;
            sense[i] = 'G';
            sprintf(rowname[i], "link(%d)", i);
        }

        rmatbeg[N] = 2 * N;
        for (int i = 0; i < N; i++) {
            rmatind[2 * N + i] = i + N;
            rmatval[2 * N + i] = 1.0;
        }
        rhs[N] = 1;
        sense[N] = 'E';
        sprintf(rowname[N], "con");

        status = CPXaddrows(env, lp, 0, numrows, numnz, rhs, sense, rmatbeg, rmatind, rmatval, NULL, rowname);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free_stringarray(rowname, numrows);
        free(rmatbeg);
        free(rmatind);
        free(rmatval);

        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qindrow[] = { 0, 0 };
        int qindcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];

        //x^2 <= yz
        for (int i = 0; i < N; i++) {
            qindrow[0] = i;
            qindcol[0] = 2 * N + i;
            qindrow[1] = N + i;
            qindcol[1] = N + i;
            sprintf(qcname, "qcon(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qindrow, qindcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }
   
    if (P == 1.5) {   
        int numvars = 4 * N;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 20);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1;

            vtype[i] = 'B';

            sprintf(varnames[i], "y_(%d)", i);

            obj[i + N] = 0.0;
            lb[i + N] = 0;
            ub[i + N] = 1;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "x_(%d)", i);

            obj[i + 2 * N] = quad_cost[i];
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "z_(%d)", i);

            obj[i + 3 * N] = 0.0;
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = 1;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "w_(%d)", i);
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
        int numconstrs = N + 1;
        int numnz = 3 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 30);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);
        for (int i = 0; i < N; i++) {
            cbeg[i] = 2 * i;
            cind[2 * i] = i;
            cval[2 * i] = 1.0;
            cind[2 * i + 1] = i + N;
            cval[2 * i + 1] = -1.0;

            rhs[i] = 0;
            sense[i] = 'G';
            sprintf(constrnames[i], "link(%d)", i);
        }

        cbeg[N] = 2 * N;
        for (int i = 0; i < N; i++) {
            cind[2 * N + i] = i + N;
            cval[2 * N + i] = 1.0;
        }
        rhs[N] = 1;
        sense[N] = 'E';
        sprintf(constrnames[N], "con");

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        /* add quadratic constraints */
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];
        //x^2 <= wz
        for (int i = 0; i < N; i++) {
            qrow[0] = 2 * N + i;
            qcol[0] = 3 * N + i;
            qrow[1] = N + i;
            qcol[1] = N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
        //w^2 <= xy
        for (int i = 0; i < N; i++) {
            qrow[0] = N + i;
            qcol[0] = i;
            qrow[1] = 3 * N + i;
            qcol[1] = 3 * N + i;
            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 2.5) {
        int numvars = 6 * N;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 30);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);

            obj[i + N] = 0.0;
            lb[i + N] = 0;
            ub[i + N] = 1;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "x_(%d)", i);

            obj[i + 2 * N] = quad_cost[i];
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "z_(%d)", i);

            obj[i + 3 * N] = 0.0;
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = 1;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "w1_(%d)", i);

            obj[i + 4 * N] = 0.0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = 1;
            vtype[i + 4 * N] = 'C';
            sprintf(varnames[i + 4 * N], "w2_(%d)", i);

            obj[i + 5 * N] = 0.0;
            lb[i + 5 * N] = 0;
            ub[i + 5 * N] = 1;
            vtype[i + 5 * N] = 'C';
            sprintf(varnames[i + 5 * N], "w3_(%d)", i);
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
        int numconstrs = N + 1;
        int numnz = 3 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 30);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);
        for (int i = 0; i < N; i++) {
            cbeg[i] = 2 * i;
            cind[2 * i] = i;
            cval[2 * i] = 1.0;
            cind[2 * i + 1] = i + N;
            cval[2 * i + 1] = -1.0;

            rhs[i] = 0;
            sense[i] = 'G';
            sprintf(constrnames[i], "link(%d)", i);
        }

        cbeg[N] = 2 * N;
        for (int i = 0; i < N; i++) {
            cind[2 * N + i] = i + N;
            cval[2 * N + i] = 1.0;
        }
        rhs[N] = 1;
        sense[N] = 'E';
        sprintf(constrnames[N], "con");

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        /* add quadratic constraints */
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];

        //x^2 <= w2w3
        for (int i = 0; i < N; i++) {
            qrow[0] = 4 * N + i;
            qcol[0] = 5 * N + i;
            qrow[1] = N + i;
            qcol[1] = N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
        //w3^2 <= yz
        for (int i = 0; i < N; i++) {
            qrow[0] = i;
            qcol[0] = 2 * N + i;
            qrow[1] = 5 * N + i;
            qcol[1] = 5 * N + i;
            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }

        //w2^2 <= w1 x
        for (int i = 0; i < N; i++) {
            qrow[0] = N + i;
            qcol[0] = 3 * N + i;
            qrow[1] = 4 * N + i;
            qcol[1] = 4 * N + i;
            sprintf(qcname, "qcon3(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }

        //w1^2 <= xy
        for (int i = 0; i < N; i++) {
            qrow[0] = i;
            qcol[0] = N + i;
            qrow[1] = 3 * N + i;
            qcol[1] = 3 * N + i;
            sprintf(qcname, "qcon4(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 3.0) {
        int numvars = 4 * N;
        double* obj = create_double_vector(numvars);
        double* lb = create_double_vector(numvars);
        double* ub = create_double_vector(numvars);
        char* vtype = create_char_vector(numvars);
        char** varnames = create_stringarray(numvars, 30);

        for (int i = 0; i < N; i++) {
            obj[i] = fixed_cost[i];
            lb[i] = 0;
            ub[i] = 1;
            vtype[i] = 'B';
            sprintf(varnames[i], "y_(%d)", i);

            obj[i + N] = 0.0;
            lb[i + N] = 0;
            ub[i + N] = 1;
            vtype[i + N] = 'C';
            sprintf(varnames[i + N], "x_(%d)", i);

            obj[i + 2 * N] = quad_cost[i];
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            vtype[i + 2 * N] = 'C';
            sprintf(varnames[i + 2 * N], "z_(%d)", i);

            obj[i + 3 * N] = 0.0;
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = 1;
            vtype[i + 3 * N] = 'C';
            sprintf(varnames[i + 3 * N], "w_(%d)", i);
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
        int numconstrs = N + 1;
        int numnz = 3 * N;
        double* rhs = create_double_vector(numconstrs);
        char* sense = create_char_vector(numconstrs);
        char** constrnames = create_stringarray(numconstrs, 30);
        int* cbeg = create_int_vector(numconstrs);
        int* cind = create_int_vector(numnz);
        double* cval = create_double_vector(numnz);
        for (int i = 0; i < N; i++) {
            cbeg[i] = 2 * i;
            cind[2 * i] = i;
            cval[2 * i] = 1.0;
            cind[2 * i + 1] = i + N;
            cval[2 * i + 1] = -1.0;

            rhs[i] = 0;
            sense[i] = 'G';
            sprintf(constrnames[i], "link(%d)", i);
        }

        cbeg[N] = 2 * N;
        for (int i = 0; i < N; i++) {
            cind[2 * N + i] = i + N;
            cval[2 * N + i] = 1.0;
        }
        rhs[N] = 1;
        sense[N] = 'E';
        sprintf(constrnames[N], "con");

        status = CPXaddrows(env, lp, 0, numconstrs, numnz, rhs, sense, cbeg, cind, cval, NULL, constrnames);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");
        free(rhs);
        free(sense);
        free_stringarray(constrnames, numconstrs);
        free(cbeg);
        free(cind);
        free(cval);

        /* add quadratic constraints */
        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qrow[] = { 0, 0 };
        int qcol[] = { 0, 0 };
        double qval[] = { 1, -1 };
        char qcname[30];

        //x^2 <= wy
        for (int i = 0; i < N; i++) {
            qrow[0] = i;
            qcol[0] = 3 * N + i;
            qrow[1] = N + i;
            qcol[1] = N + i;
            sprintf(qcname, "qcon1(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
        //w^2 <= xz
        for (int i = 0; i < N; i++) {
            qrow[0] = N + i;
            qcol[0] = 2 * N + i;
            qrow[1] = 3 * N + i;
            qcol[1] = 3 * N + i;
            sprintf(qcname, "qcon2(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qrow, qcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    if (P == 0) {//Kleinrock average delay function
        int numcols = N * 5;
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

            obj[i + N] = 0.0;
            lb[i + N] = 0;
            ub[i + N] = 1;
            xctype[i + N] = 'C';
            sprintf(colname[i + N], "x_(%d)", i);

            obj[i + 2 * N] = quad_cost[i];
            lb[i + 2 * N] = 0;
            ub[i + 2 * N] = 1;
            xctype[i + 2 * N] = 'C';
            sprintf(colname[i + 2 * N], "z_(%d)", i);

            obj[i + 3 * N] = 0;
            lb[i + 3 * N] = 0;
            ub[i + 3 * N] = 1;
            xctype[i + 3 * N] = 'C';
            sprintf(colname[i + 3 * N], "w1_(%d)", i);

            obj[i + 4 * N] = 0;
            lb[i + 4 * N] = 0;
            ub[i + 4 * N] = 1;
            xctype[i + 4 * N] = 'C';
            sprintf(colname[i + 4 * N], "w2_(%d)", i);
        }

        status = CPXnewcols(env, lp, numcols, obj, lb, ub, xctype, colname);
        if (status)
            fprintf(stderr, "CPXnewcols failed.\n");

        free(obj);
        free(lb);
        free(ub);
        free(xctype);
        free_stringarray(colname, numcols);

        int numrows = 3 * N + 1;
        int numnz = 3 * N + 2 * 3 * N;
        double* rhs = create_double_vector(numrows);
        char* sense = create_char_vector(numrows);
        char** rowname = create_stringarray(numrows, 30);
        int* rmatbeg = create_int_vector(numrows);
        int* rmatind = create_int_vector(numnz);
        double* rmatval = create_double_vector(numnz);

        for (int i = 0; i < N; i++) {
            rmatbeg[i] = 2 * i;
            rmatind[2 * i] = i;
            rmatval[2 * i] = 1.0;
            rmatind[2 * i + 1] = i + N;
            rmatval[2 * i + 1] = -1.0;

            rhs[i] = 0;
            sense[i] = 'G';
            sprintf(rowname[i], "link(%d)", i);
        }

        rmatbeg[N] = 2 * N;
        for (int i = 0; i < N; i++) {
            rmatind[2 * N + i] = i + N;
            rmatval[2 * N + i] = 1.0;
        }
        rhs[N] = 1;
        sense[N] = 'E';
        sprintf(rowname[N], "con");

        //w1 = y-x
        for (int i = 0; i < N; i++) {
            rmatbeg[N + 1 + i] = 3 * N  + 3 * i;

            rmatind[3 * N + 3 * i] = i;
            rmatval[3 * N + 3 * i] = 1.0;
            rmatind[3 * N + 3 * i + 1] = i + N;
            rmatval[3 * N + 3 * i + 1] = -1.0;
            rmatind[3 * N + 3 * i + 2] = i + 3 * N;
            rmatval[3 * N + 3 * i + 2] = -1.0;

            rhs[N + 1 + i] = 0;
            sense[N + 1 + i] = 'E';
            sprintf(rowname[N + 1 + i], "w1(%d)", i);
        }

        //w2 = z-x
        for (int i = 0; i < N; i++) {
            rmatbeg[2 * N + 1 + i] = 6 * N + 3 * i;

            rmatind[6 * N + 3 * i] = i + 2 * N;
            rmatval[6 * N + 3 * i] = 1.0;
            rmatind[6 * N + 3 * i + 1] = i + N;
            rmatval[6 * N + 3 * i + 1] = -1.0;
            rmatind[6 * N + 3 * i + 2] = i + 4 * N;
            rmatval[6 * N + 3 * i + 2] = -1.0;

            rhs[2 * N + 1 + i] = 0;
            sense[2 * N + 1 + i] = 'E';
            sprintf(rowname[2 * N + 1 + i], "w2(%d)", i);
        }

        status = CPXaddrows(env, lp, 0, numrows, numnz, rhs, sense, rmatbeg, rmatind, rmatval, NULL, rowname);
        if (status)
            fprintf(stderr, "CPXaddrows failed.\n");

        free(rhs);
        free(sense);
        free_stringarray(rowname, numrows);
        free(rmatbeg);
        free(rmatind);
        free(rmatval);

        int qnumnz = 2;
        double qrhs = 0.0;
        char qsense = 'G';
        int qindrow[] = { 0, 0 };
        int qindcol[] = { 0, 0 };
        double qval[] = { 1, -1};
        char qcname[30];

        //x^2 <= (z-x)(y-x) -> x^2 <= w1 w2
        for (int i = 0; i < N; i++) {
            qindrow[0] = 3 * N + i;
            qindcol[0] = 4 * N + i;

            qindrow[1] = N + i;
            qindcol[1] = N + i;

            sprintf(qcname, "qcon(%d)", i);
            status = CPXaddqconstr(env, lp, 0, qnumnz, qrhs, qsense, NULL, NULL, qindrow, qindcol, qval, qcname);
            if (status)
                fprintf(stderr, "CPXaddrows failed.\n");
        }
    }

    // 设置求解参数 单线程 1000s
    //CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
    CPXsetintparam(env, CPX_PARAM_THREADS, 1); // Nber of threads to use
    CPXsetdblparam(env, CPX_PARAM_EPINT, 0);
    CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9);
    CPXsetintparam(env, CPXPARAM_MIP_Strategy_MIQCPStrat, 1);// tell CPLEX to solve a QCP relaxation of the model at each node

    ////检查模型
    //status = CPXwriteprob(env, lp, "SOCP.lp", NULL);
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
    CPXsetdblparam(env, CPX_PARAM_TILIM, 1000 - RootRuntime); // time limit
    start = clock();
    if (status = CPXmipopt(env, lp)) {
        printf("Unable to optimize the SOCP reformulation\n");
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