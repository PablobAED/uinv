################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := --cmd_file="syscfg/board.opt" --cmd_file="syscfg/device_cmd.opt" --cmd_file="syscfg/c2000ware_libraries.opt" 
GEN_CMDS__FLAG := -l"syscfg/device_cmd.cmd" 

ORDERED_OBJS += \
"./syscfg/board.obj" \
"./syscfg/device.obj" \
"./syscfg/device_cmd.obj" \
"./syscfg/c2000ware_libraries.obj" \
"./f28003x_codestartbranch.obj" \
"./uinv_comms.obj" \
"./uinv_dcac.obj" \
"./uinv_dcdc.obj" \
"./uinv_main.obj" \
"./libraries/DCL/DCL_DF22_C1.obj" \
"./libraries/sfra/sfra_gui_scicomms_driverlib.obj" \
$(GEN_CMDS__FLAG) \
-lsfra_f32_tmu_eabi.lib \
-lrts2800_fpu32_fast_supplement_eabi.lib \
-lc2000ware_libraries.cmd.genlibs \
-ldriverlib.lib \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include libraries/DCL/subdir_vars.mk
-include libraries/sfra/subdir_vars.mk
-include subdir_rules.mk
-include libraries/DCL/subdir_rules.mk
-include libraries/sfra/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
uinv_F280039C_vFinal.out 

EXE_OUTPUTS__QUOTED += \
"uinv_F280039C_vFinal.out" 


# All Target
all: $(OBJS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "uinv_F280039C_vFinal.out"

# Tool invocations
uinv_F280039C_vFinal.out: $(OBJS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcrc -O4 --opt_for_speed=2 --fp_mode=relaxed --define=DEBUG --define=RAM --float_operations_allowed=32 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi -z -m"uinv_F280039C_vFinal.map" --heap_size=0x100 --stack_size=0x200 --warn_sections -i"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib" -i"C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device" -i"C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device/driverlib/ccs/Release" -i"C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/sfra" -i"C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/FPUfastRTS" -i"C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/RELEASE/syscfg" -i"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="uinv_F280039C_vFinal_linkInfo.xml" --entry_point=code_start --rom_model -o "uinv_F280039C_vFinal.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "syscfg\board.obj" "syscfg\device.obj" "syscfg\device_cmd.obj" "syscfg\c2000ware_libraries.obj" "f28003x_codestartbranch.obj" "uinv_comms.obj" "uinv_dcac.obj" "uinv_dcdc.obj" "uinv_main.obj" "libraries\DCL\DCL_DF22_C1.obj" "libraries\sfra\sfra_gui_scicomms_driverlib.obj" 
	-$(RM) "syscfg\board.d" "syscfg\device.d" "syscfg\device_cmd.d" "syscfg\c2000ware_libraries.d" "uinv_comms.d" "uinv_dcac.d" "uinv_dcdc.d" "uinv_main.d" "libraries\sfra\sfra_gui_scicomms_driverlib.d" 
	-$(RM) "f28003x_codestartbranch.d" "libraries\DCL\DCL_DF22_C1.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

