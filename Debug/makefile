################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./Lucky13.obj" \
"./console_task.obj" \
"./distance_task.obj" \
"./line_task.obj" \
"./motor_task.obj" \
"./startup_ccs.obj" \
"./switch_task.obj" \
"./drivers/buttons.obj" \
"./drivers/rgb.obj" \
"./third_party/FreeRTOS/Source/list.obj" \
"./third_party/FreeRTOS/Source/queue.obj" \
"./third_party/FreeRTOS/Source/tasks.obj" \
"./third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.obj" \
"./third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.obj" \
"./third_party/FreeRTOS/Source/portable/MemMang/heap_2.obj" \
"./utils/uartstdio.obj" \
"./utils/ustdlib.obj" \
"../freertos_demo_ccs.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"/home/arc/ti/tivaware_c_series_2_1_4_178/driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include drivers/subdir_vars.mk
-include third_party/FreeRTOS/Source/subdir_vars.mk
-include third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/subdir_vars.mk
-include third_party/FreeRTOS/Source/portable/MemMang/subdir_vars.mk
-include utils/subdir_vars.mk
-include subdir_rules.mk
-include drivers/subdir_rules.mk
-include third_party/FreeRTOS/Source/subdir_rules.mk
-include third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/subdir_rules.mk
-include third_party/FreeRTOS/Source/portable/MemMang/subdir_rules.mk
-include utils/subdir_rules.mk
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
Lucky13.out 

EXE_OUTPUTS__QUOTED += \
"Lucky13.out" 


# All Target
all: Lucky13.out

# Tool invocations
Lucky13.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --ual -z -m"freertos_demo_ccs.map" --stack_size=512 --heap_size=0 -i"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/lib" -i"/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="Lucky13_linkInfo.xml" --rom_model -o "Lucky13.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "Lucky13.obj" "console_task.obj" "distance_task.obj" "line_task.obj" "motor_task.obj" "startup_ccs.obj" "switch_task.obj" "drivers/buttons.obj" "drivers/rgb.obj" "third_party/FreeRTOS/Source/list.obj" "third_party/FreeRTOS/Source/queue.obj" "third_party/FreeRTOS/Source/tasks.obj" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.obj" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.obj" "third_party/FreeRTOS/Source/portable/MemMang/heap_2.obj" "utils/uartstdio.obj" "utils/ustdlib.obj" 
	-$(RM) "Lucky13.d" "console_task.d" "distance_task.d" "line_task.d" "motor_task.d" "startup_ccs.d" "switch_task.d" "drivers/buttons.d" "drivers/rgb.d" "third_party/FreeRTOS/Source/list.d" "third_party/FreeRTOS/Source/queue.d" "third_party/FreeRTOS/Source/tasks.d" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.d" "third_party/FreeRTOS/Source/portable/MemMang/heap_2.d" "utils/uartstdio.d" "utils/ustdlib.d" 
	-$(RM) "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"/home/arc/ti/ccs1280/ccs/utils/tiobj2bin/tiobj2bin" "Lucky13.out" "Lucky13.bin" "/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armofd" "/home/arc/ti/ccs1280/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armhex" "/home/arc/ti/ccs1280/ccs/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

