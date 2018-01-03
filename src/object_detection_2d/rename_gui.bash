for i in `ls include/object_detection_2d`
do
	sed 's/DefinitionGUI/GUI/g
	s/DEFINITION_GUI/GUI/g
	s/definition_gui/gui/g' include/object_detection_2d/$i >include/object_detection_2d/new_$i
	mv include/object_detection_2d/new_$i include/object_detection_2d/$i
done

for i in `ls src`
do
	sed 's/DefinitionGUI/GUI/g
	s/definition_gui/gui/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
