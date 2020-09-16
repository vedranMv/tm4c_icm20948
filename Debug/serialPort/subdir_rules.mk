################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
serialPort/%.obj: ../serialPort/%.cpp $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"G:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.8.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="F:/Documents/workspaces/workspace_v7/tm4c_icm20948" --include_path="F:/Documents/workspaces/workspace_v7/tm4c_icm20948/mpu9250" --include_path="F:/Documents/workspaces/workspace_v7/tm4c_icm20948/mpu9250/Invn" --include_path="F:/Programming/ti/TivaWare_C_Series-2.1.4.178" --include_path="F:/Programming/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="G:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.8.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="serialPort/$(basename $(<F)).d_raw" --obj_directory="serialPort" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


