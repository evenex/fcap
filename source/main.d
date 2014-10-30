import std.process;
import std.stdio;

void main ()
	{/*...}*/
		auto x = executeShell (``);
		writeln (x.status);
	}
