#include "def.h"

int main(int argc, char* argv[]) {

	if (argc < 5 || argc > 5) {
		usage(argv[0]);//参数过少 报错 提示用法
	}

	//读取算例类型
	P = atof(argv[4]);	
	printf("Instance Parameter: %.1f\n", P);//打印提示

	//read input file name
	sprintf(input_text, "Inputs/");//将字符串读入输入文件名数组，在stdio.h中定义
	strcat(input_text, argv[1]);//argv[1]读取的是程序名后的第一个参数 也就是input_file_name 将文件路径与输入名称连接起来

	//read output file name
	sprintf(output_text, "Results/");//将字符串读入输出文件名数组
	strcat(output_text, argv[2]);//argv[2]读取的是程序名后的第二个参数 也就是output_file_name 将文件路径与输入名称连接起来

	printf("Input: %s\nOutput: %s\n", input_text, output_text);//打印提示	

	input = open_file(input_text, "r");//open_file自定义函数 
	int num_inst;
	fscanf(input, "%d", &num_inst);//从文件流中读取算例个数 是文件流里第一个数据

	//清空文件
	output = open_file(output_text, "w+");
	fclose(output);
	for (int i = 0; i < num_inst; i++) {
	//for (int i = 0; i < 1; i++) {//运行一个算例测试
		fscanf(input, "%s", &instance); //从文件流中读取算例名称
		//打印开始求解标记
		printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\nSolving %s\n", instance);

		//读取算例
		int length = (int)strlen(argv[1]);
		int i;
		for (i = 0; i < length - 4; i++)
			dirname[i] = argv[1][i];
		dirname[i] = '/';
		dirname[i + 1] = '\0';

		read_instance(instance);

		//打开输出文件
		output = open_file(output_text, "a+");
		fprintf(output, "Instance: %s;\n", instance);
		
		//选择求解方法
		if (strcmp(argv[3], "AP2R") == 0)
			AP2R();// 利用CPLEX求解MISOCP模型
		else if (strcmp(argv[3], "BD") == 0)
			BendersDecomposition();
		else if (strcmp(argv[3], "SOCP") == 0)
			SOCP();
		else
			fprintf(stderr, "solve_method must be AP2R or BD or SOCP\n");		

		//打印结束标记
		printf("\nFinished solving %s\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", instance);
		
		fclose(output);

		//释放内存
		free_memory();

		//睡眠3秒
		Sleep(3000);
	}
	fclose(input);
	return 0;
}