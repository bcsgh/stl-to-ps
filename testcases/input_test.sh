EXE=$1
SRC=$2
STDOUT=$GTEST_TMP_DIR/stdout
STDERR=$GTEST_TMP_DIR/stderr
OUTPUT=$GTEST_TMP_DIR/out

$EXE 1>$STDOUT 2>$STDERR \
	--noparser_debug \
	--output=$OUTPUT \
	--script=$2      \
	--name=test      \

RC=$?

if [ $RC -ne 1 ];
then
	echo >&2 Expected '"'$SRC'"' to fail with RC=1
	echo >&2 RC=$RC
	echo >&2 STDERR  ================================================
	cat >&2 $STDERR
	echo >&2 STDOUT  ================================================
	cat >&2 $STDOUT
	exit 1
fi

FAILED=0

while read TARGET;
do
	if ! grep $STDERR -e "$TARGET" -q;
	then
		echo >&2 failed to find expected error:
		echo >&2 $TARGET
		FAILED=1
	fi
done < <(grep $SRC -nHe // | sed "s@\(:[0-9]*:\).*//@\1.*@")


if [ $FAILED -ne 0 ];
then 
	if [ -s $STDERR ];
	then
		echo >&2 STDERR  ================================================
		cat >&2 $STDERR
	fi
	if [ -s $STDOUT ];
	then
		echo >&2 STDOUT  ================================================
		cat >&2 $STDOUT
	fi
	exit 1
fi

exit 0