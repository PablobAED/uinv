################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-863105391: ../uinv.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1280/ccs/utils/sysconfig_1.21.0/sysconfig_cli.bat" --script "C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/uinv.syscfg" -o "syscfg" -s "C:/ti/C2000Ware_DigitalPower_SDK_5_03_00_00__all/C2000Ware_DigitalPower_SDK_5_03_00_00/c2000ware/.metadata/sdk.json" -d "F28003x" --package 100PZ --part F28003x_100PZ --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-863105391 ../uinv.syscfg
syscfg/board.h: build-863105391
syscfg/board.cmd.genlibs: build-863105391
syscfg/board.opt: build-863105391
syscfg/board.json: build-863105391
syscfg/pinmux.csv: build-863105391
syscfg/epwm.dot: build-863105391
syscfg/device.c: build-863105391
syscfg/device.h: build-863105391
syscfg/adc.dot: build-863105391
syscfg/device_cmd.cmd: build-863105391
syscfg/device_cmd.c: build-863105391
syscfg/device_cmd.h: build-863105391
syscfg/device_cmd.opt: build-863105391
syscfg/device_cmd.cmd.genlibs: build-863105391
syscfg/c2000ware_libraries.cmd.genlibs: build-863105391
syscfg/c2000ware_libraries.opt: build-863105391
syscfg/c2000ware_libraries.c: build-863105391
syscfg/c2000ware_libraries.h: build-863105391
syscfg/clocktree.h: build-863105391
syscfg: build-863105391

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcrc -O4 --opt_for_speed=2 --fp_mode=relaxed --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device/driverlib" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/DCL" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/power_measurement" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/sfra" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/spll" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/dlog" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/rampgen" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/FPUfastRTS" --include_path="C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=DEBUG --define=RAM --float_operations_allowed=32 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/RELEASE/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

f28003x_codestartbranch.obj: C:/ti/C2000Ware_DigitalPower_SDK_5_03_00_00__all/C2000Ware_DigitalPower_SDK_5_03_00_00/c2000ware/device_support/f28003x/common/source/f28003x_codestartbranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcrc -O4 --opt_for_speed=2 --fp_mode=relaxed --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device/driverlib" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/DCL" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/power_measurement" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/sfra" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/spll" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/dlog" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/rampgen" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/FPUfastRTS" --include_path="C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=DEBUG --define=RAM --float_operations_allowed=32 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/RELEASE/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcrc -O4 --opt_for_speed=2 --fp_mode=relaxed --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/device/driverlib" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/DCL" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/power_measurement" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/sfra" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/spll" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/dlog" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/utilities/rampgen" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/libraries/FPUfastRTS" --include_path="C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=DEBUG --define=RAM --float_operations_allowed=32 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/usuario/OneDrive - Jose Miguel Sanz Alcaine/CodeComposer/workspace_uinv_final_auto/uinv_F280039C_vFinal/RELEASE/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


