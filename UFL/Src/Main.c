#include "def.h"

//��1��C ���Թ涨 main �����Ĳ���ֻ�������������涨 argc ���������ͱ�����argv ������ָ���ַ�����ָ�����顣
//��2��argc ���������ܵĲ���������
//��3��char* argv[] ��ָ�����飬�����е�ÿ��Ԫ�ض��� char* ���ͣ���������ÿ��Ԫ�ض���ָ��һ���ַ�����
// (4) argvָ������ ��0���ַ����ǳ�������ƣ��Ժ�Ĳ����������к�������û�����Ĳ�����
int main(int argc, char* argv[]) {//argv[]�Ĳ���//����һ�����ڲ����εķ�ʽ���� (1)�һ���Ŀ�����ҵ������������ (2)�ڵ���-��������� ������ļ���������ַ�������  //��������cmd���д���
	//Read command line arguments ����ĸ����� progname input_file_name output_file_name [Heuristic_Level] ������������progname input_file_name output_file_name
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

	
	// r ��ֻ���ļ������ļ�������ڡ�r + �򿪿ɶ�д���ļ������ļ�������ڡ�
	 // w ��ֻд�ļ������ļ��������ļ����ݻ���ʧ�����ļ��������������ļ���w + �򿪿ɶ�д�ļ������ļ��������ļ����ݻ���ʧ�����ļ��������������ļ���
	 // a �Ը��ӵķ�ʽ��ֻд�ļ������ļ������ڣ���Ὠ�����ļ�������ļ����ڣ�д������ݻᱻ�ӵ��ļ�β�����ļ�ԭ�ȵ����ݻᱻ������
	 // a + �Ը��ӷ�ʽ�򿪿ɶ�д���ļ������ļ������ڣ���Ὠ�����ļ�������ļ����ڣ�д������ݻᱻ�ӵ��ļ�β�󣬼��ļ�ԭ�ȵ����ݻᱻ������

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
		read_instance(instance);

		//������ļ�
		output = open_file(output_text, "a+");
		fprintf(output, "Instance: %s;\n", instance);
				

		//ѡ����ⷽ��
		if (strcmp(argv[3], "BD") == 0)
			BendersDecomposition();
		else if (strcmp(argv[3], "SOCP") == 0)
			SOCP();
		else if (strcmp(argv[3], "PC") == 0)
			PC();
		else
			fprintf(stderr, "solve_method must be BD or SOCP\n");

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