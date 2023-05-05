#pragma once

// Add this macro to make the class singleton
#define MAKE_SINGLETON(classname)\
	public:\
	static classname& Instance()\
	{\
		static classname singleton;\
		return singleton;\
	}\
	private:\
	classname() = default;\
	classname(const classname &) = delete;\
	const classname & operator=(const classname &) = delete;