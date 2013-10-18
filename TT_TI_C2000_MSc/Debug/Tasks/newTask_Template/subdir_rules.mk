################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Tasks/newTask_Template/newTask.obj: ../Tasks/newTask_Template/newTask.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff -g --include_path="C:/ti/ccsv5/tools/compiler/c2000_6.1.0/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="Tasks/newTask_Template/newTask.pp" --obj_directory="Tasks/newTask_Template" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


