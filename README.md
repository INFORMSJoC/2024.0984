[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)](https://pubsonline.informs.org/journal/ijoc)

# Perspective Benders Decomposition with Applications to Fixed-Charge Nonlinear Resource Allocation

This archive is distributed in association with the [INFORMS Journal on
Computing](https://pubsonline.informs.org/journal/ijoc) under the [MIT License](LICENSE).

The software and data in this repository are a snapshot of the software and data
that were used in the research reported on in the paper [Perspective Benders Decomposition with Applications to Fixed-Charge Nonlinear Resource Allocation](https://doi.org/10.1287/ijoc.2024.0984) by K. Yang, G. Song, R. Wang, H. Yang and R. Leus.

## Cite

To cite the contents of this repository, please cite both the paper and this repo, using their respective DOIs.

https://doi.org/10.1287/ijoc.2024.0984

https://doi.org/10.1287/ijoc.2024.0984.cd

Below is the BibTex for citing this snapshot of the repository.

```
@misc{Yang2025,
  author =        {K. Yang and G. Song and R. Wang and H. Yang and R. Leus},
  publisher =     {INFORMS Journal on Computing},
  title =         {Perspective {B}enders Decomposition with Applications to Fixed-Charge Nonlinear Resource Allocation},
  year =          {2025},
  doi =           {10.1287/ijoc.2024.0984.cd},
  url =           {https://github.com/INFORMSJoC/2024.0984},
  note =          {Available for download at \url{https://github.com/INFORMSJoC/2024.0984}},
}
```

## Description

This repository provides the data and implementation accompanying the paper:  
**_Perspective Benders Decomposition with Applications to Fixed-Charge Nonlinear Resource Allocation_**.

The implementation focuses on solving two classes of fixed-charge nonlinear resource allocation problems using Perspective Benders Decomposition:

- **Generalized Sensor Placement Problem (GSPP)**
- **Generalized Uncapacitated Facility Location Problem (GUFLP)**

## Data

- The GSPP instances are sourced from the [SPP instance library](https://commalab.di.unipi.it/datasets/RDR/).
- The GUFLP instances are sourced from the [UFL benchmark library](https://resources.mpi-inf.mpg.de/departments/d1/projects/benchmarks/UflLib/).

Please refer to the original websites or our paper for detailed information about the instance authors and sources.

Due to potential licensing issues related to redistributing benchmark instances, we do **not** include the actual instance files in the `Data` directories of each subproject. Instead, users can download the datasets directly from the original websites and place them into the appropriate folders. For example:

- Copy the `2000-h` series instances for the SPP problem into `./SP/Data/2000-h/`
- Copy the `ga250` series instances for the UFL problem into `./UFL/Data/`

This setup will allow the code to run without modification.


## Code Structure

The repository contains two subdirectories, each corresponding to a problem:

- **`SP/`**: Contains source code for solving the GSPP.
- **`UFL/`**: Contains source code for solving the GUFLP.

Each subdirectory includes the following folders:

- **`Data/`**: Directory for storing the instance files required to run the models.
- **`Input/`**: Contains text files, each of which can be passed as the `input_file_name` argument to the executable. These files list the instances of the same size, including the number of instances and their filenames.
- **`Results/`**: Each file here can be used as the `output_file_name` argument. The directory stores solution results for each group of instances.
- **`Src/`**: Contains the C source code for the corresponding problem.

## Dependencies

- **C/C++ Compiler**
  - Windows: Visual Studio 2022 (preferred)
- **IBM ILOG CPLEX [version 22.1.1]**
  - A general mixed integer programming solver
  - Called through the program using its **Callable Library API**
  - Make sure CPLEX environment variables (`INCLUDE`, `LIB`) are correctly configured for compilation

## Compilation

### Windows (Recommended)

1. Open the solution file `MIRAP.sln` in **Visual Studio 2022**
2. Select the build configuration: **x64** and **Release**
3. Build the solution

After successful compilation, two executable files will be generated in:
```
./x64/Release/
```

- `SP.exe` → Solver for the Generalized Sensor Placement Problem  
- `UFL.exe` → Solver for the Generalized Uncapacitated Facility Location Problem

## Usage

### Syntax (example with UFL.exe):

Execute the program as follows:

```
./UFL input_file_name output_file_name [Method] [Problem parameter]
```

#### Arguments:

- `input_file_name`: The input data file
- `output_file_name`: The output result file
- `[Method]`: Solution approach, one of:
  - `BD`: Benders Decomposition
  - `AP2R`: Approximated Projected Perspective Reformulation (**SP.exe only**)
  - `SOCP`: Mixed-Integer Second-Order Cone Program (MISOCP)
  - `PC`: Used for solving qUFL, specifically when `p = 2.0`. Although the theoretical framework applies to **GSPP** and **GUFL**, this method is only **partially implemented** for qUFL. The implementation was **preliminary and unsatisfactory**, and hence results using this method were **not reported in the paper**. 
- `[Problem parameter]`: Type of allocation cost function:
  - `1.5`: Rational power functions (c·xᵖ where p = 1.5)
  - `2.0`: Rational power functions (c·xᵖ where p = 2.0)
  - `2.5`: Rational power functions (c·xᵖ where p = 2.5)
  - `3.0`: Rational power functions (c·xᵖ where p = 3.0)
  - `0`: Delay function c·x / (1 - x/k), where 0 < k ≤ 1
  - `-1`: Exponential function c·(e^(–k·x) – 1), where k > 0
  - `-2`: Logarithmic function –c·log(1 + k·x), where k > 0

### Example

./UFL ga250.txt ga250_2.0_BD.txt BD 2.0

## Replicating

To replicate the experimental results as reported in the paper:

1. Copy all instance files from `./SP/Data/` and `./UFL/Data/` into the shared folder `./x64/Release/Data/`.

2. Navigate to the executable directory:
```
cd ./x64/Release/
```
3. Run the provided batch script to execute all test cases:
```
./run.bat
```
All raw results will be written to the `Results/` directory.

4. Process and analyze the results using the provided Python scripts:
```
./python ReadData.py # Generates Result.xls 
```

The file `Result.xls` contains detailed results for each instance under each algorithm.

```
./python AnalysisData.py # Generates ResultAnalysis.xls
```
The file `ResultAnalysis.xls` provides aggregated statistical summaries for each set of instances, as reported in the paper.

Final Excel reports will appear in the same directory as the Python scripts.

## Note

To ensure a fair comparison with existing literature, **we use default algorithm parameters without tuning the `in-out` Benders cut stabilization parameters**. Preliminary tests suggest that simple tuning of these parameters can yield significant performance improvements.