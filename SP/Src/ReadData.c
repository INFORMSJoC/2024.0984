#include "def.h"

//usage()函数 提供程序使用方法
void usage(const char* progname) {//将程序名作为输入参数调用 输出为空
	//Print a usage message to stderr and abort.
	fprintf(stderr,//fprintf()在stdio.h中定义，格式化输出到流文件中，stderr 在stdio.h中定义，标准错误输出文件流
		"Usage: %s  input_file_name output_file_name \n", progname);//输出程序的用法
	exit(2);//输入参数有误 强制退出程序 //在stdlib.h中定义
}

//打开文件函数
FILE* open_file(const char* name, const char* mode) {
	FILE* file;
	if ((file = fopen(name, mode)) == NULL) {
		printf("\nError: Failed to open file\n");
		exit(8);
	}
	return file;
}

//创造整数列向量
int* create_int_vector(int columns) {
	int* ptr;//该指针指向整数变量
	if ((ptr = (int*)calloc(columns, sizeof(int))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}

//创造实数列向量
double* create_double_vector(int dim) {
	double* ptr;
	if ((ptr = (double*)calloc(dim, sizeof(double))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}
//申请字符向量内存
char* create_char_vector(int dim) {
	char* ptr;//该指针指向整数变量
	if ((ptr = (char*)calloc(dim, sizeof(char))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}

//创造字符串数组 每个数组有n个字符串，每个字符串有m个字符
char** create_stringarray(int n, int m) {
	char** ptr;
	int i;
	ptr = (char**)calloc(n, sizeof(char*));
	for (i = 0; i < n; i++) {
		ptr[i] = (char*)calloc(m, sizeof(char));
	}
	return ptr;
}

//释放字符串数组内存
void free_stringarray(char** ptr, int n) {
	int i;
	for (i = 0; i < n; i++) {
		free(ptr[i]);
	}
	free(ptr);
}

//读取算例函数
void read_instance(const char* name) {
	FILE* inst;//算例文件流
	char path[100];//算例文件路径变量
	sprintf(path, "Data/");//写入文件路径
	strcat(path, dirname);//连接路径名
	strcat(path, name);//合并文件路径
	inst = open_file(path, "r");//打开算例文件流

	fscanf(inst, "%d", &N);
	// 初始化内存
	initialize_memory();
	//读取数据
	for (int i = 0; i < N; i++) {
		fscanf(inst, "%lf", &fixed_cost[i]);
	}
	for (int i = 0; i < N; i++) {
		fscanf(inst, "%lf", &quad_cost[i]);
	}
	fclose(inst);
}

//初始化内存
void initialize_memory(void) {
	fixed_cost = create_double_vector(N);
	quad_cost = create_double_vector(N);

	X = create_double_vector(N+1);//主问题的解
	Stabilizing_Point = create_double_vector(N);
	Separation_Point = create_double_vector(N);

	benders_cut = (BENDERSCUT*)malloc(sizeof(BENDERSCUT));
	benders_cut->rhs = create_double_vector(1);
	benders_cut->sense = create_char_vector(1);
	benders_cut->beg = create_int_vector(1);
	benders_cut->ind = create_int_vector(N + 1);
	benders_cut->val = create_double_vector(N + 1);

	benders_cut->sense[0] = 'G';
	benders_cut->beg[0] = 0;
	for (int i = 0; i < N+1; i++) {
		benders_cut->ind[i] = i;
		benders_cut->val[i] = 0;//每次分离cut要求的
	}	
	benders_cut->rhs[0] = 0;//每次分离cut要求的

	
	x = create_double_vector(N);//子问题的解
	x_ = create_double_vector(N);
	alpha = create_double_vector(N);
	subgradient = create_double_vector(N);
	a = create_double_vector(N);
	b = create_double_vector(N);
	Rlist = create_int_vector(N);

	globvarctype = create_char_vector(N+1);
	globvarind = create_int_vector(N+1);	

	current_sol1 = create_double_vector(N + 1);
	current_sol2 = create_double_vector(N + 1);
	Rc = create_double_vector(N + 1);
	fixed_plants = create_int_vector(N);
	unfixed_index = create_int_vector(N);
	Y = create_double_vector(N + 1);

	//求解归约后的子问题
	S = (int*)malloc(sizeof(int) * N);
	S1 = (int*)malloc(sizeof(int) * N);
	S2 = (int*)malloc(sizeof(int) * N);
}

//释放内存
void free_memory(void) {
	free(fixed_cost);
	free(quad_cost);

	free(X);
	free(Stabilizing_Point);
	free(Separation_Point);

	free(benders_cut->rhs);
	free(benders_cut->sense);
	free(benders_cut->beg);
	free(benders_cut->ind);
	free(benders_cut->val);
	free(benders_cut);

	free(x);
	free(x_);
	free(alpha);
	free(subgradient);
	free(Rlist);
	free(a);
	free(b);

	free(globvarctype);
	free(globvarind);

	free(Rc);
	free(current_sol1);
	free(current_sol2);
	free(fixed_plants);
	free(unfixed_index);
	free(Y);

	free(S);
	free(S1);
	free(S2);
}