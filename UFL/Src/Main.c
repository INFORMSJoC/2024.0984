#include "def.h"

//（1）C 语言规定 main 函数的参数只能有两个，还规定 argc 必须是整型变量，argv 必须是指向字符串的指针数组。
//（2）argc 是命令行总的参数个数。
//（3）char* argv[] 是指针数组，数组中的每个元素都是 char* 类型，即数组中每个元素都会指向一个字符串。
// (4) argv指针数组 第0个字符串是程序的名称，以后的参数。命令行后面跟的用户输入的参数。
int main(int argc, char* argv[]) {//argv[]的参数//方法一：用内部传参的方式传入 (1)右击项目名，找到最下面的属性 (2)在调试-命令参数： 输入除文件名以外的字符串参数  //方法二：cmd进行传参
	//Read command line arguments 最多四个参数 progname input_file_name output_file_name [Heuristic_Level] 最少三个参数progname input_file_name output_file_name
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

	
	// r 打开只读文件，该文件必须存在。r + 打开可读写的文件，该文件必须存在。
	 // w 打开只写文件，若文件存在则文件内容会消失。若文件不存在则建立该文件。w + 打开可读写文件，若文件存在则文件内容会消失。若文件不存在则建立该文件。
	 // a 以附加的方式打开只写文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾，即文件原先的内容会被保留。
	 // a + 以附加方式打开可读写的文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾后，即文件原先的内容会被保留。

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
		read_instance(instance);

		//打开输出文件
		output = open_file(output_text, "a+");
		fprintf(output, "Instance: %s;\n", instance);
				

		//选择求解方法
		if (strcmp(argv[3], "BD") == 0)
			BendersDecomposition();
		else if (strcmp(argv[3], "SOCP") == 0)
			SOCP();
		else if (strcmp(argv[3], "PC") == 0)
			PC();
		else
			fprintf(stderr, "solve_method must be BD or SOCP\n");

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