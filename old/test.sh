FILE=$(find ./ -name "*.cpp")
BITS="-m32"
WARN="	-Wall 
	-Wextra 
	-Wpointer-arith 
	-Wcast-align 
	-Wcast-qual
	-Wwrite-strings
	-Wswitch-default
	-Wswitch-enum
	-Wunreachable-code
	-save-temps
	-Werror 
	-Wno-error=unused-variable 
	-Wno-error=unused-function 
	-Wno-error=unused-parameter 
	-Wno-error=unused-but-set-variable 
	"
DIRS=""
LIBS="	-lstdc++
	-lm 
	-lnidaqmxbase
	-lcv 
	-lcxcore 
	-lhighgui
	-lgsl
	-lgslcblas
	"

if [ "$1" == "-d" ] || [ "$2" == "-d" ]	 #DEBUG
then
	MODE=debug
	OPTS=""
elif [ "$1" == "-c" ]	#COMPILE ONLY
then
	MODE=compile
	OPTS=""
elif [ "$1" == "-r" ]	#RELEASE
then
	MODE=normal
	OPTS="-O"
elif [ "$1" == "-p" ]	#PROFILE
then
	MODE=profile
	OPTS=""
elif [ "$1" == "-pp" ]	#OPTIMIZED PROFILE
then
	MODE=profile
	OPTS="-ffast-math -O"
elif [ "$1" == "-l" ] 	#LINE-BY-LINE PROFILE
then
	MODE=deep_profile
	OPTS=""
elif [ "$1" == "-ll" ] 	#OPTIMIZED LINE-BY-LINE PROFILE
then
	MODE=deep_profile
	OPTS="-ffast-math -O"
elif [ "$1" == "-cov" ]	#CODE COVERAGE
then
	MODE=coverage
	OPTS=""
elif [ "$1" == "-dep" ]	#DEPENDENCY
then
	MODE=compile
	OPTS="-MM"

else
	MODE=normal	#NORMAL EXECUTION
	OPTS=""
fi
if [ "$MODE" == "debug" ]
then
	FLAGS="-g"
elif [ "$MODE" == "profile" ]
then
	FLAGS="-pg"
elif [ "$MODE" == "deep_profile" ]
then
	FLAGS="-g -pg"
elif [ "$MODE" == "coverage" ]
then
	FLAGS="-fprofile-arcs -ftest-coverage"
elif [ "$MODE" == "compile" ]
then
	FLAGS="-g"
else
	FLAGS=""
fi
gcc -std=gnu++11 				\
	$FLAGS $BITS $OPTS $WARN	\
	$FILE 				\
	$DIRS				\
	$LIBS				\
2> log.txt
if [ -e a.out ]
then
	if [ "$1" == "-R" ]
	then
		echo -e "\t\tREBUILD SUCCESS!";
	else
		echo -e "\t\tCOMPILE SUCCESS!";
	fi
	if [ "$MODE" == "debug" ]
	then
		gdb a.out
	elif [ "$MODE" == "profile" ]
	then
		./a.out $@
		gprof -b > analysis.txt
		rm gmon.out
		gvim analysis.txt
	elif [ "$MODE" == "deep_profile" ]
	then
		./a.out $@
		gprof -lb > analysis.txt
		rm gmon.out
		less analysis.txt
	elif [ "$MODE" == "coverage" ]
	then
		./a.out $@
	else
		if [ "$MODE" == "compile" ]
		then
			echo
		else
			./a.out $@
		fi
	fi
	if [ "$MODE" == "compile" ]
	then
		echo
	else
		rm a.out
	fi
else
	echo -e "\t\tCOMPILE FAILED!"
fi
if [ "$(wc -l log.txt | grep -o [0-9]*)" -gt "0" ]
then
	cat log.txt
fi
mv *.ii ./out;
mv *.o ./out;
mv *.s ./out;
rm log.txt
