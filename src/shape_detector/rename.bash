# Check argc.
case $# in
	2) ;;
	*) echo "Usage: $0 <from> <to>"; exit 1;;
esac

from=$1
to=$2
cap_from=`echo $from | sed 's/[a-z]/\u&/g'`
cap_to=`echo   $to   | sed 's/[a-z]/\u&/g'`

# Rename headers.
for i in `ls include/$to`
do
	sed 's/'$cap_from'/'$cap_to'/g
	s/'$from'/'$to'/g' include/$to/$i >include/$to/new_$i
	mv include/$to/new_$i include/$to/$i
done

# Rename source files.
for i in `ls src`
do
	sed 's/'$cap_from'/'$cap_to'/g
	s/'$from'/'$to'/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
