################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
third_party/FreeRTOS/Source/list.obj: /home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/list.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/home/arc/ti/Lucky13" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/examples/boards/ek-tm4c123gxl" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/include" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="third_party/FreeRTOS/Source/$(basename $(<F)).d_raw" --obj_directory="third_party/FreeRTOS/Source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

third_party/FreeRTOS/Source/queue.obj: /home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/queue.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/home/arc/ti/Lucky13" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/examples/boards/ek-tm4c123gxl" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/include" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="third_party/FreeRTOS/Source/$(basename $(<F)).d_raw" --obj_directory="third_party/FreeRTOS/Source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

third_party/FreeRTOS/Source/tasks.obj: /home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/tasks.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/home/arc/ti/Lucky13" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/examples/boards/ek-tm4c123gxl" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/include" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS" --include_path="/home/arc/ti/tivaware_c_series_2_1_4_178/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="third_party/FreeRTOS/Source/$(basename $(<F)).d_raw" --obj_directory="third_party/FreeRTOS/Source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


