#ifndef __Get_dt_HPP
#define __Get_dt_HPP

#include <Windows.h>
#include <time.h>

class __dt {

public:

	__dt() {

	}// __dt()

	double get() {

		update_Time();

		return dt;

	}// double get()
private:

	SYSTEMTIME t_now, t_last;
	
	double dt;

	void update_Time() {

		t_last = t_now;
		GetSystemTime(&t_now);
		dt =	(t_now.wMinute - t_last.wMinute) * 60 + 
			(t_now.wSecond - t_last.wSecond) * 1000 + 
			t_now.wMilliseconds - t_last.wMilliseconds;

	}// int update_Time()

};



#endif //__Get_dt_HPP