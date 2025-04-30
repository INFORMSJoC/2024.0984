#ifndef DEF_H
#define DEF_H

#include <stdio.h>
#include <time.h>
#include <stdbool.h>//引入bool类型
#include <string.h>
#include <stdlib.h>
#include <ilcplex/cplex.h>
#include <math.h>
#include <windows.h>

#define EPS 1e-3
#define MAX(x, y) ( ((x) >= (y) ) ? (x) : (y) )

int node_id;        // Remember the Node Id
int last_id;		// Id of last branch&bound node visited
int last_id_visits;	// # of times that we have visited last B&B node

//Benders cut
typedef struct {
	char* sense;
	double* rhs;
	int* beg;
	int* ind;
	double* val;
} BENDERSCUT;

BENDERSCUT* benders_cut;

//FILE* 文件流指针 在stdio.h里面定义
FILE* input;//输入文件
FILE* output;//输出文件
double RootRuntime;

double P;//算例参数
char dirname[10];
//求解子问题所需变量
double* x;
double* x_;//x/y的临时变量
double* alpha;
double* subgradient;
double* a;
double* b;
int* Rlist;
//求解归约后的子问题
int* S;
int* S1;
int* S2;

//修改变量属性
char* globvarctype;
int* globvarind;
char  errmsg[CPXMESSAGEBUFSIZE];

//文件名数组
char input_text[50];//输入文件名
char output_text[50];//输出文件名

//算例信息变量
char instance[30];//算例名称数组
//算例数据
int N;//the number of the facility: i
double* fixed_cost;//the opening cost
double* quad_cost;//

//获取求解信息
double LowerBound;
double* X;
double* Stabilizing_Point;
double* Separation_Point;

//数学启发式使用
double UpperBound;
double UpperBound1;
double UpperBound2;
double* Rc;
double* current_sol1;
double* current_sol2;
double* current_best_sol;

//削减主问题规模使用
double LowerBound_Try;
int Num_unfixed;
double pre_fix_sum;
int* fixed_plants;
int* unfixed_index;
double* Y;//已经固定的向量

//实用函数
void usage(const char* progname);
FILE* open_file(const char* name, const char* mode);

void read_instance(const char* name);

void initialize_memory(void);
void free_memory(void);

char* create_char_vector(int dim);
double* create_double_vector(int dim);
int* create_int_vector(int columns);
char** create_stringarray(int n, int m);
void free_stringarray(char** ptr, int n);

void AP2R(void);
void SOCP(void);

//Benders分解
void BendersDecomposition(void);
void MasterProblem(CPXENVptr* envptr, CPXLPptr* lpptr);
void Root_node_solve(CPXENVptr env, CPXLPptr lp);
void Cut_Stabilization(CPXENVptr env, CPXLPptr lp, double lambda);
double GBD_Separator(double* Separation_Point);
void SetBranchandCutParam(CPXENVptr env, CPXLPptr lp);
int CPXPUBLIC myusercutcallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p);
int CPXPUBLIC mylazycutcallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p);
void solve_ip_and_get_solution_info(CPXENVptr env, CPXLPptr lp);

//数学启发式
void MatHeuristic(CPXENVptr env, CPXLPptr lp);
void Heuristic_nonfix();
int close_min_facility_and_nonfix();
double Evaluate_Objective(double* current_sol);
void reduced_cost_var_elimination(CPXENVptr env, CPXLPptr lp);

#endif    //  DEF_H