SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility;C:\Users\ha'se'e\AppData\Local\Microsoft\WindowsApps;;C:\Users\ha'se'e\AppData\Local\Programs\Microsoft VS Code\bin
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x00B71B00
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\system_stm32f10x.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\max6675.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\pwm.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\tim3.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\fuzzypid.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\dc_pid.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\delay.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\sys.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\core_cm3.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\obj\startup_stm32f10x_md._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\misc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_spi.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f10x_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\OBJ\USART.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\OBJ\USART.axf" --i32combined --output "..\OBJ\USART.hex"
