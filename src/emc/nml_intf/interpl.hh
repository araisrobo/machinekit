/********************************************************************
* Description: interpl.hh
*   Mechanism for building lists of arbitrary NML messages, used by
*   the canonical interface and interpreter to pass planned sequences
*   to the HME.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef INTERP_LIST_HH
#define INTERP_LIST_HH

#define MAX_NML_COMMAND_SIZE 1000

// these go on the interp list
struct NML_INTERP_LIST_NODE {
    int line_number;		// line number it was on
    int call_level;             // call_level for interp._setup.call_level
    int remap_level;            // remap_level for interp._setup.remap_level

    union _dummy_union {
	int i;
	long l;
	double d;
	float f;
	long long ll;
	long double ld;
    } dummy;			// paranoid alignment variable.

    union _command_union {
	char commandbuf[MAX_NML_COMMAND_SIZE];	// the NML command;
	int i;
	long l;
	double d;
	float f;
	long long ll;
	long double ld;
    } command;
};

// here's the interp list itself
class NML_INTERP_LIST {
  public:
    NML_INTERP_LIST();
    ~NML_INTERP_LIST();

    int set_line_number(int line);
    int set_interp_params(int c_level, int r_level);
    int get_line_number();
    int get_call_level();
    int get_remap_level();
    int append(NMLmsg &);
    int append(NMLmsg *);
    NMLmsg *get();              //!< update contents from head and delete it from list
    NMLmsg *update_current();   //!< update contents from current_node
    int move_next();            //!< move current node to next node
    int move_last();            //!< move current node to last node
    int move_head();            //!< move current node to head
    int move_tail();            //!< move current node to tail
    NMLmsg *search(int lineno, int c_level, int r_level);
    NMLmsg *get_by_lineno(int lineno);
    NMLmsg *get_next_lineno(int lineno);
    NMLmsg *get_last_lineno(int lineno);
    bool is_eol();              //!< is current_node at End-Of-List?
    bool is_bol();              //!< is current_node at Begin-Of-List?
    void clear();
    void print();
    void set_z(double z);       //!< set Z value of all history nodes TO-BE-FIXED with remapped python program

    int len();

  private:
    class LinkedList * linked_list_ptr;
    NML_INTERP_LIST_NODE temp_node;	// filled in and put on the list
    int next_line_number;       //!< line number used to fill temp_node
    int next_call_level;        //!< call_level used to fill temp_node
    int next_remap_level;       //!< remap_level used to fill temp_node
    int line_number;		//!< line number of node from get()/update_current()
    int call_level;             //!< call_level of node from get()/update_current()
    int remap_level;            //!< remap_level of node from get()/update_current()
};

extern NML_INTERP_LIST interp_list;	/* NML Union, for interpreter */

#endif
