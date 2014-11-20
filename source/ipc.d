module ipc;

import std.stdio;
import std.file;

bool read_ipc_flag (string flag_path)
	{/*...}*/
		return exists (flag_path);
	}
void write_ipc_flag (string flag_path, bool value)
	{/*...}*/
		if (value)
			File (flag_path, `w`).write (``);
		else if (flag_path.exists)
			remove (flag_path);
	}
void await_ipc_flag (string flag_path, bool delegate() while_waiting = null)
	{/*...}*/
		while (read_ipc_flag (flag_path) == false)
			if (while_waiting !is null && while_waiting () == false)
				return;

		write_ipc_flag (flag_path, false);
	}
