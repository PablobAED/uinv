################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
libraries/sfra/%.obj: ../libraries/sfra/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcrc -O4 --opt_for_speed=2 --fp_mode=relaxed --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device/driverlib" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/DCL" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/power_measurement" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/sfra" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/spll" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/dlog" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/rampgen" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/FPUfastRTS" --include_path="C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=DEBUG --define=RAM --float_operations_allowed=32 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="libraries/sfra/$(basename $(<F)).d_raw" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/RELEASE/syscfg" --obj_directory="libraries/sfra" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


