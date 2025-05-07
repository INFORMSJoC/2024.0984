#include "def.h"

//usage()���� �ṩ����ʹ�÷���
void usage(const char* progname) {//����������Ϊ����������� ���Ϊ��
	//Print a usage message to stderr and abort.
	fprintf(stderr,//fprintf()��stdio.h�ж��壬��ʽ����������ļ��У�stderr ��stdio.h�ж��壬��׼��������ļ���
		"Usage: %s  input_file_name output_file_name \n", progname);//���������÷�
	exit(2);//����������� ǿ���˳����� //��stdlib.h�ж���
}

//���ļ�����
FILE* open_file(const char* name, const char* mode) {
	FILE* file;
	if ((file = fopen(name, mode)) == NULL) {
		printf("\nError: Failed to open file\n");
		exit(8);
	}
	return file;
}

//��������������
int* create_int_vector(int columns) {
	int* ptr;//��ָ��ָ����������
	if ((ptr = (int*)calloc(columns, sizeof(int))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}

//����ʵ��������
double* create_double_vector(int dim) {
	double* ptr;
	if ((ptr = (double*)calloc(dim, sizeof(double))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}
//�����ַ������ڴ�
char* create_char_vector(int dim) {
	char* ptr;//��ָ��ָ����������
	if ((ptr = (char*)calloc(dim, sizeof(char))) == NULL) {
		printf("\nError: Memoria insuficiente\n");
		exit(8);
	}
	return ptr;
}

//�����ַ������� ÿ��������n���ַ�����ÿ���ַ�����m���ַ�
char** create_stringarray(int n, int m) {
	char** ptr;
	int i;
	ptr = (char**)calloc(n, sizeof(char*));
	for (i = 0; i < n; i++) {
		ptr[i] = (char*)calloc(m, sizeof(char));
	}
	return ptr;
}

//�ͷ��ַ��������ڴ�
void free_stringarray(char** ptr, int n) {
	int i;
	for (i = 0; i < n; i++) {
		free(ptr[i]);
	}
	free(ptr);
}

//��ȡ��������
void read_instance(const char* name) {
	FILE* inst;//�����ļ���
	char path[100];//�����ļ�·������
	sprintf(path, "Data/");//д���ļ�·��
	strcat(path, dirname);//����·����
	strcat(path, name);//�ϲ��ļ�·��
	inst = open_file(path, "r");//�������ļ���

	fscanf(inst, "%d", &N);
	// ��ʼ���ڴ�
	initialize_memory();
	//��ȡ����
	for (int i = 0; i < N; i++) {
		fscanf(inst, "%lf", &fixed_cost[i]);
	}
	for (int i = 0; i < N; i++) {
		fscanf(inst, "%lf", &quad_cost[i]);
	}
	fclose(inst);
}

//��ʼ���ڴ�
void initialize_memory(void) {
	fixed_cost = create_double_vector(N);
	quad_cost = create_double_vector(N);

	X = create_double_vector(N+1);//������Ľ�
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
		benders_cut->val[i] = 0;//ÿ�η���cutҪ���
	}	
	benders_cut->rhs[0] = 0;//ÿ�η���cutҪ���

	
	x = create_double_vector(N);//������Ľ�
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

	//����Լ���������
	S = (int*)malloc(sizeof(int) * N);
	S1 = (int*)malloc(sizeof(int) * N);
	S2 = (int*)malloc(sizeof(int) * N);
}

//�ͷ��ڴ�
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