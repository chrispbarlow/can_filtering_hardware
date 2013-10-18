################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/c2000_6.1.0

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
-l"rts2800_fpu32_fast_supplement.lib" \
-l"rts2800_fpu32.lib" \
"./CAN_Exchange/CAN_Rx_Filter_global.obj" \
"./Lib/CAN/CAN.obj" \
"./Lib/SCI/SCI.obj" \
"./Lib/TI/DSP2833x_usDelay.obj" \
"./Lib/TI/DSP2833x_SysCtrl.obj" \
"./Lib/TI/DSP2833x_Sci.obj" \
"./Lib/TI/DSP2833x_PieVect.obj" \
"./Lib/TI/DSP2833x_PieCtrl.obj" \
"./Lib/TI/DSP2833x_GlobalVariableDefs.obj" \
"./Lib/TI/DSP2833x_ECan.obj" \
"./Lib/TI/DSP2833x_DefaultIsr.obj" \
"./Lib/TI/DSP2833x_CpuTimers.obj" \
"./Lib/TI/DSP2833x_CodeStartBranch.obj" \
"./Lib/TI/DSP2833x_ADC_cal.obj" \
"./Tasks/controlSCI/controlSCI.obj" \
"./Tasks/handleCAN/handleCAN.obj" \
"./Tasks/newTask_Template/newTask.obj" \
"./Tasks/receiveCAN/receiveCAN.obj" \
"./scheduler/schedulerConfig.obj" \
"./scheduler/scheduler.obj" \
"./TT_Main.obj" \
-l"IQmath_fpu32.lib" \
-l"libc.a" \
"../F28335.cmd" \
"C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include scheduler/subdir_vars.mk
-include Tasks/receiveCAN/subdir_vars.mk
-include Tasks/newTask_Template/subdir_vars.mk
-include Tasks/handleCAN/subdir_vars.mk
-include Tasks/controlSCI/subdir_vars.mk
-include Lib/TI/subdir_vars.mk
-include Lib/SCI/subdir_vars.mk
-include Lib/CAN/subdir_vars.mk
-include CAN_Exchange/subdir_vars.mk
-include subdir_rules.mk
-include scheduler/subdir_rules.mk
-include Tasks/receiveCAN/subdir_rules.mk
-include Tasks/newTask_Template/subdir_rules.mk
-include Tasks/handleCAN/subdir_rules.mk
-include Tasks/controlSCI/subdir_rules.mk
-include Lib/TI/subdir_rules.mk
-include Lib/SCI/subdir_rules.mk
-include Lib/CAN/subdir_rules.mk
-include CAN_Exchange/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Example_2833xCpuTimer.out

# Tool invocations
Example_2833xCpuTimer.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data -z --stack_size=0x200 -m"Example_2833xCpuTimer.map" --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/include" -i"C:/Code/MSc_project/MSc_Hardware/TT_TI_C2000_MSc" -i"C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/lib" -i"C:/ti/controlSUITE/libs/math/IQmath/v15c/lib" -i"C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/lib" --reread_libs --priority --verbose_diagnostics --issue_remarks --entry_point=code_start --rom_model -o "Example_2833xCpuTimer.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "Example_2833xCpuTimer.out"
	-$(RM) "TT_Main.pp" "scheduler\scheduler.pp" "scheduler\schedulerConfig.pp" "Tasks\receiveCAN\receiveCAN.pp" "Tasks\newTask_Template\newTask.pp" "Tasks\handleCAN\handleCAN.pp" "Tasks\controlSCI\controlSCI.pp" "Lib\TI\DSP2833x_CpuTimers.pp" "Lib\TI\DSP2833x_DefaultIsr.pp" "Lib\TI\DSP2833x_ECan.pp" "Lib\TI\DSP2833x_GlobalVariableDefs.pp" "Lib\TI\DSP2833x_PieCtrl.pp" "Lib\TI\DSP2833x_PieVect.pp" "Lib\TI\DSP2833x_Sci.pp" "Lib\TI\DSP2833x_SysCtrl.pp" "Lib\SCI\SCI.pp" "Lib\CAN\CAN.pp" "CAN_Exchange\CAN_Rx_Filter_global.pp" 
	-$(RM) "TT_Main.obj" "scheduler\scheduler.obj" "scheduler\schedulerConfig.obj" "Tasks\receiveCAN\receiveCAN.obj" "Tasks\newTask_Template\newTask.obj" "Tasks\handleCAN\handleCAN.obj" "Tasks\controlSCI\controlSCI.obj" "Lib\TI\DSP2833x_ADC_cal.obj" "Lib\TI\DSP2833x_CodeStartBranch.obj" "Lib\TI\DSP2833x_CpuTimers.obj" "Lib\TI\DSP2833x_DefaultIsr.obj" "Lib\TI\DSP2833x_ECan.obj" "Lib\TI\DSP2833x_GlobalVariableDefs.obj" "Lib\TI\DSP2833x_PieCtrl.obj" "Lib\TI\DSP2833x_PieVect.obj" "Lib\TI\DSP2833x_Sci.obj" "Lib\TI\DSP2833x_SysCtrl.obj" "Lib\TI\DSP2833x_usDelay.obj" "Lib\SCI\SCI.obj" "Lib\CAN\CAN.obj" "CAN_Exchange\CAN_Rx_Filter_global.obj" 
	-$(RM) "Lib\TI\DSP2833x_ADC_cal.pp" "Lib\TI\DSP2833x_CodeStartBranch.pp" "Lib\TI\DSP2833x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
