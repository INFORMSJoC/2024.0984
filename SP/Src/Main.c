#include "def.h"

int main(int argc, char* argv[]) {

	if (argc < 5 || argc > 5) {
		usage(argv[0]);//�������� ���� ��ʾ�÷�
	}

	//��ȡ��������
	P = atof(argv[4]);	
	printf("Instance Parameter: %.1f\n", P);//��ӡ��ʾ

	//read input file name
	sprintf(input_text, "Inputs/");//���ַ������������ļ������飬��stdio.h�ж���
	strcat(input_text, argv[1]);//argv[1]��ȡ���ǳ�������ĵ�һ������ Ҳ����input_file_name ���ļ�·��������������������

	//read output file name
	sprintf(output_text, "Results/");//���ַ�����������ļ�������
	strcat(output_text, argv[2]);//argv[2]��ȡ���ǳ�������ĵڶ������� Ҳ����output_file_name ���ļ�·��������������������

	printf("Input: %s\nOutput: %s\n", input_text, output_text);//��ӡ��ʾ	

	input = open_file(input_text, "r");//open_file�Զ��庯�� 
	int num_inst;
	fscanf(input, "%d", &num_inst);//���ļ����ж�ȡ�������� ���ļ������һ������

	//����ļ�
	output = open_file(output_text, "w+");
	fclose(output);
	for (int i = 0; i < num_inst; i++) {
	//for (int i = 0; i < 1; i++) {//����һ����������
		fscanf(input, "%s", &instance); //���ļ����ж�ȡ��������
		//��ӡ��ʼ�����
		printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\nSolving %s\n", instance);

		//��ȡ����
		int length = (int)strlen(argv[1]);
		int i;
		for (i = 0; i < length - 4; i++)
			dirname[i] = argv[1][i];
		dirname[i] = '/';
		dirname[i + 1] = '\0';

		read_instance(instance);

		//������ļ�
		output = open_file(output_text, "a+");
		fprintf(output, "Instance: %s;\n", instance);
		
		//ѡ����ⷽ��
		if (strcmp(argv[3], "AP2R") == 0)
			AP2R();// ����CPLEX���MISOCPģ��
		else if (strcmp(argv[3], "BD") == 0)
			BendersDecomposition();
		else if (strcmp(argv[3], "SOCP") == 0)
			SOCP();
		else
			fprintf(stderr, "solve_method must be AP2R or BD or SOCP\n");		

		//��ӡ�������
		printf("\nFinished solving %s\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", instance);
		
		fclose(output);

		//�ͷ��ڴ�
		free_memory();

		//˯��3��
		Sleep(3000);
	}
	fclose(input);
	return 0;
}