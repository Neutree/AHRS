#include "App.h"

App app;


/**
 * main
 * @return main return value
 */
int main()
{
	app.HardwareInit();
	app.SoftwareInit();
	while(1)
	{
		app.Loop();
	}
}
