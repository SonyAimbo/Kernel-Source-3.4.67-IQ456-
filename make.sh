#!/bin/bash

# Автор: Гайдарлы Алексей (shaR)
# 4pda: djextremes
# Дата создания: 23 сентября 2016
# Дата обновления: 23 сентября 2016
#
# Скрипт предназначен для сборки ядра 3.4.67 по желанию можно переделать и под 3.10 кому нужно
# напишите в qms я сделаю. Надеюсь что вам как и мне облегчил жизнь данный скрипт - так как глупо
# тратить время на то что-бы вводить одни и те же команды вручную. Ну и лень :)

# ***********************************************************
#      О Т Р Е Д А К Т И Р О В А Т Ь   П О Д   С Е Б Я
# ***********************************************************
# Имя проекта, смотреть в mediatek/config
# mt65xx и т.п. не является названием проекта, это платформа (не путать)
PROJECT_NAME="fly_iq456"
# Тулчейн
TOOLCHAIN_PATH="/home/shar/android/source/kk_fly_iq456/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin/arm-linux-androideabi-"

# Имя ядра
export KBUILD_BUILD_USER="shaR"
export KBUILD_BUILD_HOST="MK"

# Arch
export ARCH=arm
export SUBARCH=arm

# Укажите путь к java-6-oracle
export JAVA_HOME=/usr/lib/jvm/java-6-oracle
export USE_CCACHE=1

# ***********************************************************
#                     Н Е   Т Р О Г А Т Ь
# ***********************************************************
#
SCRITP_FOLDER="script"
LOG_FOLDER="log"
SHOW_TIME=$(date +"%T")
KERN_IMG="out/target/product/${PROJECT_NAME}/kernel_${PROJECT_NAME}.bin"

# Цвета
blue='\033[0;34m'
cyan='\033[0;36m'
yellow='\033[0;33m'
red='\033[0;31m'
nocol='\033[0m'

bold=$(tput bold)
normal=$(tput sgr0)

# ***********************************************************
#                     M A I N   M E N U
# ***********************************************************
function MainMenu() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "----------------------"
	echo "  M A I N - M E N U "
	echo "----------------------"
  echo "1. КОМПИЛЯЦИЯ"
  echo "2. СНЯТЬ ЛОГИ"
  echo "3. FASTBOOT"
  echo "4. ПЕРЕПАКОВКА"
  echo "5. НАСТРОЙКА ОС"
  echo "6. УДАЛИТЬ ПАПКУ OUT"
  echo ""
  echo "0. ВЫЙТИ"
  echo "----------------------"
  read -p "Выберите действие[0-6] " mMenu

  case $mMenu in
      0)
        exit 0
      ;;
      1)
        echo " "
        # Удаляем папку out перед компиляцией
        echo -e "*********************************************************"
        echo -e "$cyan $bold $SHOW_TIME $normal $nocol - Удаляю папку OUT "
        rm -rf out

        # Тулчейн
        export CROSS_COMPILE=$TOOLCHAIN_PATH

        # Собираем
        echo -e "$cyan $bold $SHOW_TIME $nocol $nocol - Начинаю сборку"
        echo -e "*********************************************************"
        sleep 3

        ./mk ${PROJECT_NAME} n k

        # Проверяем собралось ли ядро, если нет то выводим сообщение об ошибке
        # или об успешной сборке
        echo -e "*********************************************************"

        if ! [ -a $KERN_IMG ]; then
          echo -e "$red $bold ERROR:$normal $nocol Не удалось собрать ядро!Пожалуйста исправьте ошибки!";
          echo -e "*********************************************************"
          exit 25
          sleep 3
        else
          BUILD_END=$(date +"%T")

          echo -e "$yellow $bold DONE:$normal $nocolПоздравляю, файл kernel_${PROJECT_NAME}.bin успешно собран! $BUILD_END - $SHOW_TIME"
          echo -e "*********************************************************"
          sleep 3
        fi
        MainMenu
      ;;
      2)
        mkdir_log
        LogMenu
      ;;
      3)
        FastbootMenu
      ;;
      4)
        RepackMenu
      ;;
      5)
        SettingOS
      ;;
      *)
        echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function LogMenu() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "----------------------"
	echo "    L O G - M E N U   "
	echo "----------------------"
  echo "1. СНЯТЬ ЛОГИ DMESG"
  echo "2. СНЯТЬ ЛОГИ KMSG"
  echo "3. СНЯТЬ GPIO"
  echo "4. СНЯТЬ ЛОГИ LOGCAT"
  echo ""
  echo "0. НАЗАД"
  echo "----------------------"
  read -p "Выберите действие[0-4] " LogMenu

  case $LogMenu in
    0)
      MainMenu
    ;;
    1)
      adb shell su -c dmesg > $SCRITP_FOLDER/$LOG_FOLDER/dmesg.log
      sleep 1
      LogMenu
    ;;
    2)
      adb shell su -c cat /proc/kmsg > $SCRITP_FOLDER/$LOG_FOLDER/kmsg.log
      sleep 1
      LogMenu
    ;;
    3)
      GPIOLog
    ;;
    4)
      LogMenu
    ;;
    *)
      echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function GPIOLog() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "----------------------"
	echo "    GPIO - M E N U   "
	echo "----------------------"
  echo "1. СНЯТЬ ЛОГИ GPIORead"
  echo "2. СНЯТЬ ЛОГИ MTGPIO"
  echo ""
  echo "0. НАЗАД"
  echo "----------------------"
  read -p "Выберите действие[0-2] " gpioMenu

  case $gpioMenu in
      0)
        LogMenu
      ;;
      1)
        echo -e "*********************************************************"
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Снимаю логи GPIORead"
        echo -e "*********************************************************"
        sudo adb shell GPIORead > $SCRITP_FOLDER/$LOG_FOLDER/GPIORead.log

        GPIO_LOG=$SCRITP_FOLDER/$LOG_FOLDER/GPIORead.log

        # Проверяем наличие ошибок
        if grep "/system/bin/sh: GPIORead: not found" $GPIO_LOG; then
          echo -e "*********************************************************"
          echo -e "$red $bold ERROR:$normal $nocolПоложите файл GPIORead из папки script/files в телефон по пути /system/bin и выставьте ему права (777)"
          echo -e "*********************************************************"
        else
          if grep "GPIORead: can't execute: Permission denied" $GPIO_LOG; then
            echo -e "*********************************************************"
            echo -e "$red $bold ERROR:$normal $nocolВыставьте файлу GPIORead по пути /system/bin права (777)"
            echo -e "*********************************************************"
          else
            if grep "Meta_GPIO_Init() fail" $GPIO_LOG; then
              echo -e "*********************************************************"
              echo -e "$red $bold ERROR:$normal $nocol GPIO_Init() fail проверьте mediatek/platform/mt6582/external/meta/gpio/meta_gpio_test.c"
              echo -e "*********************************************************"
            else
              echo -e "*********************************************************"
              echo -e "[$cyan $bold $SHOW_TIME $normal $nocol]$yellow $bold DONE:$normal $nocolЛоги успешно сняты, файл GPIORead.log лежит в папке script/log"
              echo -e "*********************************************************"
            fi
          fi
        fi
        sleep 2
        GPIOLog
      ;;
      2)
        echo -e "*********************************************************"
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Снимаю логи GPIORead"
        echo -e "*********************************************************"

        sudo adb shell cat sys/devices/virtual/misc/mtgpio/pin > $SCRITP_FOLDER/$LOG_FOLDER/mtgpio.log

        MTGPIO_LOG=$SCRITP_FOLDER/$LOG_FOLDER/mtgpio.log

        if grep "PIN:" $MTGPIO_LOG; then
          echo -e "*********************************************************"
          echo -e "[$cyan $bold $SHOW_TIME $normal $nocol]$yellow $bold DONE:$normal $nocolЛоги успешно сняты, файл mtgpio.log лежит в папке script/log"
          echo -e "*********************************************************"
        else
          echo -e "*********************************************************"
          echo -e "$red $bold ERROR:$normal $nocolНеизвестная ошибка"
          echo -e "*********************************************************"
        fi
        sleep 2
        GPIOLog
      ;;
      *)
        echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function FastbootMenu() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "----------------------"
	echo " F B O O T - M E N U  "
	echo "----------------------"
  echo "1. ПРОШИТЬ BOOT.IMG"
  echo "2. ПРОШИТЬ RECOVERY.IMG"
  echo "3. ПРОШИТЬ UPDATE.ZIP"
  echo ""
  echo "0. НАЗАД"
  echo "----------------------"
  read -p "Выберите действие[0-3] " fbMenu

  case $fbMenu in
      0)
        MainMenu
      ;;
      1)
        repackBoot

        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Перезагружаю в fastboot и прошиваю"
        adb reboot bootloader
        fastboot flash boot $SCRITP_FOLDER/boot.img
        fastboot reboot
        echo -e "*********************************************************"
        sleep 2
        FastbootMenu
      ;;
      2)
        repackRecovery

        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Перезагружаю в fastboot и прошиваю"
        adb reboot bootloader
        fastboot flash recovery $SCRITP_FOLDER/recovery.img
        fastboot reboot
        echo -e "*********************************************************"
        sleep 2
        FastbootMenu
      ;;
      3)
        echo "В разработке"
        sleep 2
        FastbootMenu
      ;;
      *)
        echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function RepackMenu() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "----------------------"
	echo "R E P A C K - M E N U "
	echo "----------------------"
  echo "1. ПЕРЕПАКОВАТЬ BOOT.IMG"
  echo "2. ПЕРЕПАКОВАТЬ RECOVERY.IMG"
  echo "3. ПЕРЕПАКОВАТЬ UPDATE.ZIP"
  echo ""
  echo "0. НАЗАД"
  echo "----------------------"
  read -p "Выберите действие[0-3] " rpkMenuRead

  case $rpkMenuRead in
      0)
        MainMenu
      ;;
      1)
        repackBoot
        sleep 2
        RepackMenu
      ;;
      2)
        repackRecovery
        sleep 2
        RepackMenu
      ;;
      3)
        echo "В разработке"
        sleep 2
        RepackMenu
      ;;
      *)
        echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function SettingOS() {
  clear
  echo "  "
  echo "               S C R I P T   C R E A T E D   B Y   D J E X T R E M E S   "
  echo "                               V E R S I O N   0. 1. 2                   "
  echo ""
	echo "---------------------------------"
	echo "S E T T I N G S   O S - M E N U  "
	echo "---------------------------------"
  echo "1. УСТАНОВКА JAVA 6 ORACLE"
  echo ""
  echo "0. НАЗАД"
  echo "----------------------"
  read -p "Выберите действие[0-3] " rpkMenuRead

  case $rpkMenuRead in
      0)
        MainMenu
      ;;
      1)
        echo -e "*********************************************************"
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Добавляю репозиторий ppa:webupd8team/java"
        sudo add-apt-repository ppa:webupd8team/java
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Обновляю"
        sudo apt-get update
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Устанавливаю JAVA 6 ORACLE"
        sudo apt-get install oracle-java6-installer
        echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Проверяю версию JAVA"
        java -version
        echo -e "*********************************************************"
      ;;
      2)

      ;;
      3)

      ;;
      *)
        echo -e "$red $bold ERROR:$normal $nocol нет такой команды" && sleep 2
  esac
}

function repackBoot() {
  echo -e "*********************************************************"
  echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Перепаковываю boot.img"
  sleep 2

  # Проверяем наличие boot.img
  if [ -f $SCRITP_FOLDER/boot.img ]; then
    rm $SCRITP_FOLDER/boot.img
  fi

  # Проверяем наличие скомпилированного ядра
  if ! [ -f $KERN_IMG ]; then
    echo -e "$red $bold ERROR:$normal $nocol Не найден файл $KERN_IMG!";
    echo -e "*********************************************************"
    exit 25
  fi

  MTK_TOOLS_FOLDER=$SCRITP_FOLDER/utils/mtk-tools

  # Проверяем наличие files/boot.img
  if ! [ -f $SCRITP_FOLDER/files/boot.img ]; then
    echo -e "$red $bold ERROR:$normal $nocol Не найден файл $SCRITP_FOLDER/files/boot.img!";
    echo -e "*********************************************************"
    exit 25
  fi

  $MTK_TOOLS_FOLDER/unpack-MTK.pl $SCRITP_FOLDER/files/boot.img
  # Удаляю ядро
  rm boot.img-kernel.img
  $MTK_TOOLS_FOLDER/repack-MTK.pl -boot out/target/product/$PROJECT_NAME/kernel_$PROJECT_NAME.bin  boot.img-ramdisk $SCRITP_FOLDER/boot.img
  # Удаляю рамдиск
  rm -rf boot.img-ramdisk
  rm boot.img-args.txt
}

function repackRecovery() {
  echo -e "*********************************************************"
  echo -e "[$cyan $bold $SHOW_TIME $normal $nocol] Перепаковываю recovery.img"
  sleep 2

  # Проверяем наличие boot.img
  if [ -a $SCRITP_FOLDER/recovery.img ]; then
    rm $SCRITP_FOLDER/recovery.img
  fi

  rm $SCRITP_FOLDER/recovery.img

  # Проверяем наличие скомпилированного ядра
  if ! [ -a $KERN_IMG ]; then
    echo -e "$red $bold ERROR:$normal $nocol Не найден файл $KERN_IMG!";
    echo -e "*********************************************************"
    exit 25
  fi

  MTK_TOOLS_FOLDER=$SCRITP_FOLDER/utils/mtk-tools

  # Проверяем наличие files/boot.img
  if ! [ -a $SCRITP_FOLDER/files/recovery.img ]; then
    echo -e "$red $bold ERROR:$normal $nocol Не найден файл $SCRITP_FOLDER/files/recovery.img!";
    echo -e "*********************************************************"
    exit 25
  fi

  $MTK_TOOLS_FOLDER/unpack-MTK.pl $SCRITP_FOLDER/files/recovery.img
  # Удаляю ядро
  rm recovery.img-kernel.img
  $MTK_TOOLS_FOLDER/repack-MTK.pl -recovery out/target/product/$PROJECT_NAME/kernel_$PROJECT_NAME.bin recovery.img-ramdisk $SCRITP_FOLDER/recovery.img
  # Удаляю рамдиск
  rm -rf recovery.img-ramdisk
  rm recovery.img-args.txt
}

function mkdir_log() {
  if ! [ -a $SCRITP_FOLDER/log ]; then
    mkdir $SCRITP_FOLDER/log
    echo "Успешно создал папку $SCRITP_FOLDER/log"
  fi
}
MainMenu
