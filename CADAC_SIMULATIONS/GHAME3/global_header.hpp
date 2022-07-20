///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_header.hpp'
//Declares the classes: 'Module', 'Variable', 'Document', 'Event', 'Packet'
// and the stuctures 'Module'
//
//001206 Created by Peter Zipfel
//060510 Updated from F16C for CRUISE, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////

//preventing warnings in MS C++8 for not using security enhanced CRT functions 
#define _CRT_SECURE_NO_DEPRECATE

#ifndef global_header__HPP
#define global_header__HPP

#include <fstream>
#include <string>		
#include "utility_header.hpp"

using namespace std;


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Structure 'Module'
//
//Provides the structure for the modules of the simulation
//
//010107 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
struct Module
{
	string name;
	string definition;
	string initialization;
	string execution;
	string termination;
};

///////////////////////////////////////////////////////////////////////////////
//Class 'Variable'
//Establishing module-variables as type Variable
//Provides the class for the variables used in modules
//
//001125 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

class Variable 
{
private:
	char name[CHARN]; //label of variable
	char type[CHARN]; //type of variable 'int'; default is real
	double rval;	  //real value
	int ival;         //integer value
	Matrix VEC;       //3x1 vector 
	Matrix MAT;       //3x3 matrix 
	char def[CHARL];  //definition and units
	char mod[CHARN];  //module name where variable is calculated
	char role[CHARN]; //role that variable plays: 'data', 'state', 'diag', 'out'
	char out[CHARN];  //output for: 'scrn', 'plot', 'com'
	char error[2];	  //error code '*' = SAME LOCATION multiple, overwritten definitions
					  //           'A' = SAME NAME assigned to multiple locations 
public:
	Variable()
	{
		VEC.dimension(3,1);MAT.dimension(3,3);
		strcpy(name,"empty");
		error[0]=' ';error[1]='\0';
		int dum=1;
	}; 
	~Variable(){};

	//////////////////////////// Protopypes ///////////////////////////////////////
	void init(char *na,double rv,char *de,char *mo,char *ro,char *ou);
	void init(char *na,char *ty,int iv,char *de,char *mo,char *ro,char *ou);
	void init(char *na,double v1,double v2,double v3,char *de,char *mo,char *ro,char *ou);
	void init(char *na,double v11,double v12,double v13,double v21,double v22,double v23,
						double v31,double v32,double v33,char *de,char *mo,char *ro,char *ou);

	////////////////////////// Inline Functions ///////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'name' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_name(){return name;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'type' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_type(){return type;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining value (of type 'double') from module-variable array to local variable
	//Example: thrust_com=Variable::cruise[13].real();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double real(){return rval;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining value (of type 'int') from module-variable array to local variable
	//Example: mprop=Variable::cruise[10].integer();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int integer(){return ival;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining vector value (of type 'Matrix') from module-variable array to local variable
	//Example: FSPV=Variable::round3[200].vec();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Matrix vec(){return VEC;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining matrix value (of type 'Matrix') from module-variable array to local variable
	//Example: TGV=Variable::round3[22].mat();
	//
	//001226 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Matrix mat(){return MAT;}
	
	///////////////////////////////////////////////////////////////////////////
	//Four-times overloaded function gets()
	//Loads module-variable onto module-variable array
	//Overloaded for real, integer and vector variables
	//Example: cruise[10].gets(mprop);
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double gets(double rv)
	{
		rval=rv;
		return rval;
	}
	
	double gets(int iv)
	{
		ival=iv;
		return ival;
	} 
	Matrix gets_vec(Matrix VE)
	{
		VEC=VE;
		return VEC;
	} 

	Matrix gets_mat(Matrix MA)
	{
		MAT=MA;
		return MAT;
	} 

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'def' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_def(){return def;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'mod' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_mod(){return mod;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'role' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_role(){return role;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'out' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_out(){return out;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'error' code from module-variable array 
	//
	//020909 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_error(){return error;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'error' code into module-variable  
	//Error code must be single caracter
	//
	//020911 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_error(char *error_code){strcpy(error,error_code);}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Event'
//
//Provides the 'Event' class declaration
//
//010119 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Event
{
private:
	Variable *watch_variable_ptr;	//pointer to variable to be watched
	double watch_value;			//numerical value for comparison (integers will be converted later)
	char event_operator;		// < , > , =  three options of relational opertors	
	int round3_indices[NVAR];	//new variables to be read from 'round3[]'
	double round3_values[NVAR];	//new values to be given to variables 
	int round3_size;				//size (number) of variables from round3 array
	int cruise_indices[NVAR];	//new variables to be read from 'cruise[]'
	double cruise_values[NVAR];	//new values to be given to variables
	int cruise_size;				//size (number) of variables from cruise array
public:
	Event(){};
	~Event(){};

	///////////////////////////////////////////////////////////////////////////
	//Setting watch variable 
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_variable(Variable *variable)
	{
		watch_variable_ptr=variable;
	}

	///////////////////////////////////////////////////////////////////////////
	//Setting watch value
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_value(double value){watch_value=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting event criterion (relational operator)
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_operator(char criterion){event_operator=criterion;}

	///////////////////////////////////////////////////////////////////////////
	//Setting index array of 'round3' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round3_index(int element,int index){round3_indices[element]=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting values of new 'round3' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round3_value(int element,double value){round3_values[element]=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting size of new 'round3' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round3_size(int size){round3_size=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting index array of 'cruise' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_cruise_index(int element,int index){cruise_indices[element]=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting values of 'cruise' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_cruise_value(int element,double value){cruise_values[element]=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting size of new 'cruise' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_cruise_size(int size){cruise_size=size;}

	///////////////////////////////////////////////////////////////////////////
	//Getting watch variable
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Variable *get_variable(){return watch_variable_ptr;}

	///////////////////////////////////////////////////////////////////////////
	//Getting value of watch variable
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double get_value(){return watch_value;}

	///////////////////////////////////////////////////////////////////////////
	//Getting operator of event criterion
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char get_operator(){return event_operator;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index array of 'round3' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int *get_round3_indices(){return round3_indices;}

	///////////////////////////////////////////////////////////////////////////
	//Getting values of new 'round3' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double *get_round3_values(){return round3_values;}

	///////////////////////////////////////////////////////////////////////////
	//Getting size of new 'round3' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_round3_size(){return round3_size;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index array of 'cruise' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int *get_cruise_indices(){return cruise_indices;}

	///////////////////////////////////////////////////////////////////////////
	//Getting values of new 'cruise' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double *get_cruise_values(){return cruise_values;}

	///////////////////////////////////////////////////////////////////////////
	//Getting size of new 'cruise' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_cruise_size(){return cruise_size;}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Packet'
//Provides the 'Packet' class declaration
//Packets are data clusters; one for each vehicle object, used in 'combus'
//
//010206 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Packet
{
private:
	string id;			//identification of vehicle object
	int status;			//alive=1, dead=0. hit=-1 
	int ndata;			//number of module-variables in data array
	Variable *data;		//array of module-variables identified by "com" 
public:
	Packet(){};
	~Packet(){};
	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'id'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_id(string identification){id=identification;}

	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'status'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_status(int vehicle_status){status=vehicle_status;}

	///////////////////////////////////////////////////////////////////////////
	//Setting number of module variables in data
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_ndata(int vehicle_ndata){ndata=vehicle_ndata;}

	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'data'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_data(Variable *vehicle_d){data=vehicle_d;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'id' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	string get_id(){return id;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'status' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_status(){return status;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'ndata' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_ndata(){return ndata;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'data' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Variable *get_data(){return data;}
};
///////////////////////////////////////////////////////////////////////////////
//Class 'Document'
//Stores a subset of module-variable for documentation
//
//020913 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Document 
{
private:
	int doc_offset;	  //marks the offset of the last entry
	char name[CHARN]; //label of variable
	char type[CHARN]; //type of variable 'int'; default is real
	char def[CHARL];  //definition and units
	char mod[CHARN];  //module where variable is calculated		
public:
	Document()
	{
		strcpy(name,"end_array");
	}
	~Document(){};

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'doc_offset' 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_doc_offset(){return doc_offset;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'name' from module-variable 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_name(){return name;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'type' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_type(){return type;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'def' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_def(){return def;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'mod' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_mod(){return mod;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'doc_offset' 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_doc_offset(int mark){doc_offset=mark;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'name' of module-variable 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_name(char *na){strcpy(name,na);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'type' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_type(char *ty){strcpy(type,ty);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'def' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_def(char *de){strcpy(def,de);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'mod' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_mod(char *mo){strcpy(mod,mo);}	
};
#endif
