import std.json, std.file, std.stdio;

void main(){
	float[string] nyan;
	nyan["inu"] = 0.5f;
	nyan["nago"] = 0.1f;
	nyan["buma"] = 0.3f;

	auto f = File("test.json", "w");

	f.write("[\n");
	f.write("\t{\n");

	int i=0;
	foreach(string s, n; nyan){
		f.write("\t\t\"", s, "\":", n);
		i++;
		if(i!=nyan.length){
			f.write(",\n");
		}
	}

	f.write("\n\t}\n");
	f.write("]\n");

	f.close();

	auto jsonString = read("test.json");

	auto nyanson = parseJSON(jsonString);

	foreach(elem; nyanson.array){
		writeln(elem["inu"].floating);
		writeln(elem["nago"].floating);
		writeln(elem["buma"].floating);
	}


}
