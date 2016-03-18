RM := rm -rf

# All Target
all: libibs.so

OBJS += \
./ibs.o 

C_DEPS += \
./ibs.d

# Rules for building sources
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	gcc -I"../kernel" -O0 -g3 -Wall -c -fPIC -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

# Tool invocations
libibs.so: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	gcc -shared -o"libibs.so" -fPIC $(OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(C_DEPS) libibs.so
	-@echo ' '

.PHONY: all clean
