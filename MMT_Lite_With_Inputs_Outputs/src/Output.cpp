//-----------------------------------------------------------//
// File: Output.cpp
//
// This class controls the screen/file output of the simulation.
//
// Developer: Ray Sells
// Other Developers: Dennis Strickland
//-----------------------------------------------------------//

#include <string>
#include <vector>
#include <cstdio>
#include <sstream>
//#include <iomanip>

#include "Output.h"
#include "System.h"
#include "Filer.h"


Output::Output( string infile, System *sysp) {

	//Local access
	sys = sysp;

	//Input
	Filer *ff = new Filer( "./input/"+infile);
		ff->setLine0( "#Output");
		ff->iValue = 1; // token to get for value when name found

		outfile = ff->getParam( "fname");
#ifdef RT_HWIL
		ptHz = ff->getDouble( "hwil_pt");
#else
		ptHz = ff->getDouble( "pt");
#endif
		record       = ff->getInt("record");
		runFile_rec  = ff->getInt("runFile_rec");
		preTime_rec  = ff->getInt("preTime_rec");
		pt_console   = ff->getDouble( "pt_console");
		displayInput = ff->getInt( "displayInput");
		delete ff;

		pt = 1.0/ptHz;

		firstFlag = -1;

		//fp = fopen( outfile.c_str(), "w");  

}

void Output::add( string name, double *value) {
	Entry e;
	e.name = name;
	*value = 0.0;  // MAKE SURE points to a valid #, fixes lock-up!
	e.value   = value;
	e.valueff = NULL;
	e.valuei  = NULL;
	e.valueb  = NULL;
	e.type = 0;
	vEntry.push_back( e);
}

void Output::add( string name, float *value) {
	Entry e;
	e.name = name;
	*value = 0.0;  // MAKE SURE points to a valid #, fixes lock-up!
	e.value   = NULL;
	e.valueff = value;
	e.valuei  = NULL;
	e.valueb  = NULL;
	e.type = 1;
	vEntry.push_back( e);
}

void Output::add( string name, int *value) {
	Entry e;
	e.name = name;
	*value = 0;  // MAKE SURE points to a valid #, fixes lock-up!
	e.value   = NULL;
	e.valueff = NULL;
	e.valuei  = value;
	e.valueb  = NULL;
	e.type = 2;
	vEntry.push_back( e);
}

void Output::add( string name, bool *value) {
	Entry e;
	e.name = name;
	*value = 0;  // MAKE SURE points to a valid #, fixes lock-up!
	e.value   = NULL;
	e.valueff = NULL;
	e.valuei  = NULL;
	e.valueb  = value;
	e.type = 3;
	vEntry.push_back( e);
}

//-------------------------------------------------//

void Output::init() {

		if(runFile_rec == -1) { return; } //Do not write run files

		//Create Run Performance File
		stringstream mcFileStr;
		mcFileStr << "output/" << outfile << ".dat";
		//
		fp_MC = fopen( mcFileStr.str().c_str(), "w");  

		if( initCount == 0)
		{
			for( unsigned int i = 0; i < vEntry.size(); i++)
			{
				string s = vEntry[i].name;
				fprintf( fp_MC, "%s ", vEntry[i].name.c_str());
			}
			fprintf( fp_MC, "%s", "\n");
		}
		//
		mcFileStr.clear();

		//Create Header File
		// if(firstFlag == -1) {
		//   stringstream hdrFileStr;
		//   hdrFileStr << "output/hdr";
		//   //
		//   fp_HDR = fopen( hdrFileStr.str().c_str(), "w");  

		//   for( unsigned int i = 0; i < vEntry.size(); i++) {
		//     string s = vEntry[i].name;
		//     fprintf( fp_HDR, "%s\n", vEntry[i].name.c_str());
		//   }
		//   //
		//   hdrFileStr.clear();
		//   fclose(fp_HDR);
	
		//   firstFlag = 1;
		// }
}

//-------------------------------------------------//

void Output::update() {

	if( State::sample())
	{
		tick = ( double)kt++;
	}
	//
	if( State::sample( State::EVENT, 0.0)) {}

}

//-------------------------------------------------//

void Output::rpt() {

	if( State::sample( pt)) {

		if(runFile_rec == -1) { return; } //Do not write run files

		if( ((preTime_rec == -1) && (sys->t_sys >= 0.0)) || (preTime_rec == 1) ) {
			for( unsigned int i = 0; i < vEntry.size(); i++) {
				if( i > 0) {
					fprintf( fp_MC, "%s", " " );
				}
				if( vEntry[i].type == 0) {
					double vx[2] = { 0.0, *vEntry[i].value}; // for fun...
				fprintf( fp_MC, "%.12f", 1[vx]);
				}
				if( vEntry[i].type == 1) {
					fprintf( fp_MC, "%.6g", *vEntry[i].valueff);
				}
				if( vEntry[i].type == 2) {
					fprintf( fp_MC, "%d", *vEntry[i].valuei);
				}
			}
			fprintf( fp_MC, "%s", "\n");
		}
	}
}

//-------------------------------------------------//

void Output::closeFiles()
{
		if(runFile_rec == -1) { return; } //run files not written so no need to close

		fclose(fp_MC);
}

//-------------------------------------------------//

void Output::reAssignPtr(System *sysp)
{
	//Reassign Class ptr since input class is not declared yet.
	sys = sysp;
}
