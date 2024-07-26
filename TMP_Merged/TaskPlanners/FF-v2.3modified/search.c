

/*********************************************************************
 * (C) Copyright 2001 Albert Ludwigs University Freiburg
 *     Institute of Computer Science
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 *********************************************************************/


/*
 * THIS SOURCE CODE IS SUPPLIED  ``AS IS'' WITHOUT WARRANTY OF ANY KIND, 
 * AND ITS AUTHOR AND THE JOURNAL OF ARTIFICIAL INTELLIGENCE RESEARCH 
 * (JAIR) AND JAIR'S PUBLISHERS AND DISTRIBUTORS, DISCLAIM ANY AND ALL 
 * WARRANTIES, INCLUDING BUT NOT LIMITED TO ANY IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND
 * ANY WARRANTIES OR NON INFRINGEMENT.  THE USER ASSUMES ALL LIABILITY AND
 * RESPONSIBILITY FOR USE OF THIS SOURCE CODE, AND NEITHER THE AUTHOR NOR
 * JAIR, NOR JAIR'S PUBLISHERS AND DISTRIBUTORS, WILL BE LIABLE FOR 
 * DAMAGES OF ANY KIND RESULTING FROM ITS USE.  Without limiting the 
 * generality of the foregoing, neither the author, nor JAIR, nor JAIR's
 * publishers and distributors, warrant that the Source Code will be 
 * error-free, will operate without interruption, or will meet the needs 
 * of the user.
 */











/*********************************************************************
 *
 * File: search.c
 *
 * Description: implementation of routines that search the state space
 *
 *              ADL version, Goal Agenda driven
 *                           Enforced Hill-climbing enhanced with
 *                           informed Goal Added Deletion Heuristic,
 *
 *                           and, alternatively, standard Best First Search
 *
 *
 * Author: Joerg Hoffmann 2000
 *
 *********************************************************************/ 









#include "ff.h"

#include "output.h"
#include "memory.h"

#include "relax.h"
#include "search.h"














/*****************
 * LOCAL GLOBALS *
 *****************/









/* in agenda driven algorithm, the current set of goals is this
 */
State lcurrent_goals;



/* search space for EHC
 */
EhcNode *lehc_space_head, *lehc_space_end, *lehc_current_start, *lehc_current_end;



/* memory (hash table) for states that are already members
 * of the breadth - first search space in EHC
 */
EhcHashEntry_pointer lehc_hash_entry[EHC_HASH_SIZE];
int lnum_ehc_hash_entry[EHC_HASH_SIZE];
int lchanged_ehc_entrys[EHC_HASH_SIZE];
int lnum_changed_ehc_entrys;
Bool lchanged_ehc_entry[EHC_HASH_SIZE];



/* memory (hash table) for states that are already 
 * encountered by current serial plan
 */
PlanHashEntry_pointer lplan_hash_entry[PLAN_HASH_SIZE];



/* search space
 */
BfsNode *lbfs_space_head, *lbfs_space_had;



/* memory (hash table) for states that are already members
 * of the best first search space
 */
BfsHashEntry_pointer lbfs_hash_entry[BFS_HASH_SIZE];












/********************************
 * EHC FUNCTION, CALLED BY MAIN *
 ********************************/











Bool do_enforced_hill_climbing( State *start, State *end )

{

  static Bool first_call = TRUE;
  static State S, S_;
  int i, h, h_;

  if ( first_call ) {
    /* on first call, initialize plan hash table, search space, search hash table
     */
    for ( i = 0; i < PLAN_HASH_SIZE; i++ ) {
      lplan_hash_entry[i] = NULL;
    }
    /* on subsequent calls, the start is already hashed, as it's the end
     * of the previous calls
     */
    hash_plan_state( start, 0 );
    
    lehc_space_head = new_EhcNode();
    lehc_space_end = lehc_space_head;
    
    for ( i = 0; i < EHC_HASH_SIZE; i++ ) {
      lehc_hash_entry[i] = NULL;
      lnum_ehc_hash_entry[i] = 0;
      lchanged_ehc_entry[i] = FALSE;
    }
    lnum_changed_ehc_entrys = 0;

    make_state( &S, gnum_ft_conn ); 
    S.max_F = gnum_ft_conn;
    make_state( &S_, gnum_ft_conn );
    S_.max_F = gnum_ft_conn;

    make_state( &lcurrent_goals, gnum_ft_conn );
    lcurrent_goals.max_F = gnum_ft_conn;

    first_call = FALSE;
  }
  
  /* start enforced Hill-climbing
   */

  source_to_dest( &lcurrent_goals, end );  

  source_to_dest( &S, start );
  h = get_1P_and_H( &S, &lcurrent_goals );

  if ( h == INFINITY ) {
    return FALSE;
  }
  if ( h == 0 ) {
    return TRUE;
  }  
  printf("\n\nCueing down from goal distance: %4d into depth ", h);

  while ( h != 0 ) {
    if ( !search_for_better_state( &S, h, &S_, &h_ ) ) {
      return FALSE;
    }
    source_to_dest( &S, &S_ );
    h = h_;
    printf("\n                                %4d            ", h);
  }

  return TRUE;

}











/*************************************************
 * FUNCTIONS FOR BREADTH FIRST SEARCH IN H SPACE *
 *************************************************/













Bool search_for_better_state( State *S, int h, State *S_, int *h_ )

{

  static Bool first_call = TRUE;
  static State S__;

  int i, h__, depth = 0, g;
  EhcNode *tmp;

  if ( first_call ) {
    make_state( &S__, gnum_ft_conn );
    S__.max_F = gnum_ft_conn;
    first_call = FALSE;
  }

  /* don't hash states, but search nodes.
   * this way, don't need to keep states twice in memory
   */
  tmp = new_EhcNode();
  copy_source_to_dest( &(tmp->S), S);
  hash_ehc_node( tmp );

  lehc_current_end = lehc_space_head->next;
  for ( i = 0; i < gnum_H; i++ ) {
    g = result_to_dest( &S__, S, gH[i] );
    add_to_ehc_space( &S__, gH[i], NULL, g );
  }
  lehc_current_start = lehc_space_head->next;

  while ( TRUE ) {  
    if ( lehc_current_start == lehc_current_end ) {
      reset_ehc_hash_entrys();
      free( tmp );
      return FALSE;
    }
    if ( lehc_current_start->depth > depth ) {
      depth = lehc_current_start->depth;
      if ( depth > gmax_search_depth ) {
	gmax_search_depth = depth;
      }
      printf("[%d]", depth);
      fflush( stdout );
    }
    h__ = expand_first_node( h );
    if ( LESS( h__, h ) ) {
      break;
    }
  }

  reset_ehc_hash_entrys();
  free( tmp );

  extract_plan_fragment( S );

  source_to_dest( S_, &(lehc_current_start->S) );
  *h_ = h__;

  return TRUE;

}



void add_to_ehc_space( State *S, int op, EhcNode *father, int new_goal )

{

  /* see if state is already a part of this search space
   */
  if ( ehc_state_hashed( S ) ) {
    return;
  }

  if ( !lehc_current_end ) {
    lehc_current_end = new_EhcNode();
    lehc_space_end->next = lehc_current_end;
    lehc_space_end = lehc_current_end;
  }

  copy_source_to_dest( &(lehc_current_end->S), S );
  lehc_current_end->op = op;
  lehc_current_end->father = father;
  if ( !father ) {
    lehc_current_end->depth = 1;
  } else {
    lehc_current_end->depth = father->depth + 1;
  }
  lehc_current_end->new_goal = new_goal;

  hash_ehc_node( lehc_current_end );

  lehc_current_end = lehc_current_end->next;

}



int expand_first_node( int h )

{

  static Bool fc = TRUE;
  static State S_;

  int h_, i, g;

  if ( fc ) {
    make_state( &S_, gnum_ft_conn );
    S_.max_F = gnum_ft_conn;
    fc = FALSE;
  }

  h_ = get_1P_and_H( &(lehc_current_start->S), &lcurrent_goals );
    
  if ( h_ == INFINITY ) {
    lehc_current_start = lehc_current_start->next;
    return h_;
  }

  if ( lehc_current_start->new_goal != -1 &&
       new_goal_gets_deleted( lehc_current_start ) ) {
    lehc_current_start = lehc_current_start->next;
    return INFINITY;
  }

  if ( h_ < h ) {
    return h_;
  }

  for ( i = 0; i < gnum_H; i++ ) {
    g = result_to_dest( &S_, &(lehc_current_start->S), gH[i] );
    add_to_ehc_space( &S_, gH[i], lehc_current_start, g );
  }
    
  lehc_current_start = lehc_current_start->next;

  return h_;

}











/********************************************************
 * HASHING ALGORITHM FOR RECOGNIZING REPEATED STATES IN *
 * EHC BREADTH FIRST SEARCH                             *
 ********************************************************/












void hash_ehc_node( EhcNode *n )

{

  int i, sum, index;
  EhcHashEntry *h, *prev = NULL;

  sum = state_sum( &(n->S) );
  index = sum & EHC_HASH_BITS;

  h = lehc_hash_entry[index];
  if ( !h ) {
    h = new_EhcHashEntry();
    h->sum = sum;
    h->ehc_node = n;
    lehc_hash_entry[index] = h;
    lnum_ehc_hash_entry[index]++;
    if ( !lchanged_ehc_entry[index] ) {
      lchanged_ehc_entrys[lnum_changed_ehc_entrys++] = index;
      lchanged_ehc_entry[index] = TRUE;
    }
    return;
  }
  i = 0;
  while ( h ) {
    if ( i == lnum_ehc_hash_entry[index] ) {
      break;
    }
    i++;
    prev = h;
    h = h->next;
  }

  if ( h ) {
    /* current list end is still in allocated list of hash entrys
     */
    h->sum = sum;
    h->ehc_node = n;
    lnum_ehc_hash_entry[index]++;
    if ( !lchanged_ehc_entry[index] ) {
      lchanged_ehc_entrys[lnum_changed_ehc_entrys++] = index;
      lchanged_ehc_entry[index] = TRUE;
    }
    return;
  }
  /* allocated list ended; connect a new hash entry to it.
   */
  h = new_EhcHashEntry();
  h->sum = sum;
  h->ehc_node = n;
  prev->next = h;
  lnum_ehc_hash_entry[index]++;
  if ( !lchanged_ehc_entry[index] ) {
    lchanged_ehc_entrys[lnum_changed_ehc_entrys++] = index;
    lchanged_ehc_entry[index] = TRUE;
  }
  return;
      
}



Bool ehc_state_hashed( State *S )

{

  int i, sum, index;
  EhcHashEntry *h;

  sum = state_sum( S );
  index = sum & EHC_HASH_BITS;

  h = lehc_hash_entry[index];
  for ( i = 0; i < lnum_ehc_hash_entry[index]; i++ ) {
    if ( h->sum != sum ) {
      h = h->next;
      continue;
    }
    if ( same_state( &(h->ehc_node->S), S ) ) {
      return TRUE;
    }
    h = h->next;
  }

  return FALSE;

}



Bool same_state( State *S1, State *S2 ) 

{

  int i, j;

  if ( S2->num_F != S1->num_F ) {
    return FALSE;
  }

  for ( i = 0; i < S2->num_F; i++ ) {
    for ( j = 0; j < S1->num_F; j++ ) {
      if ( S1->F[j] == S2->F[i] ) {
	break;
      }
    }
    if ( j == S1->num_F ) {
      return FALSE;
    }
  }

  return TRUE;

}



int state_sum( State *S )

{

  int i, sum = 0;

  for ( i = 0; i < S->num_F; i++ ) {
    sum += gft_conn[S->F[i]].rand;
  }

  return sum;

}



void reset_ehc_hash_entrys( void )

{

  int i;

  for ( i = 0; i < lnum_changed_ehc_entrys; i++ ) {
    lnum_ehc_hash_entry[lchanged_ehc_entrys[i]] = 0;
    lchanged_ehc_entry[lchanged_ehc_entrys[i]] = FALSE;
  }
  lnum_changed_ehc_entrys = 0;

}











/***************************************************
 * FUNCTIONS FOR UPDATING THE CURRENT SERIAL PLAN, *
 * BASED ON SEARCH SPACE INFORMATION .             *
 *                                                 *
 * EMPLOY SOMEWHAT TEDIOUS HASHING PROCEDURE TO    *
 * AVOID REPEATED STATES IN THE PLAN               *
 ***************************************************/












void extract_plan_fragment( State *S )

{

  EhcNode *i;
  int ops[MAX_PLAN_LENGTH], num_ops;
  State_pointer states[MAX_PLAN_LENGTH];
  int j;
  PlanHashEntry *start = NULL, *i_ph;

  num_ops = 0;
  for ( i = lehc_current_start; i; i = i->father ) {
    if ( (start = plan_state_hashed( &(i->S) )) != NULL ) {
      for ( i_ph = start->next_step; i_ph; i_ph = i_ph->next_step ) {
	i_ph->step = -1;
      }
      gnum_plan_ops = start->step;
      break;
    }
    if ( num_ops == MAX_PLAN_LENGTH ) {
      printf("\nincrease MAX_PLAN_LENGTH! currently %d\n\n",
	     MAX_PLAN_LENGTH);
      exit( 1 );
    }
    states[num_ops] = &(i->S);
    ops[num_ops++] = i->op;
  }
  if ( !start ) {
    start = plan_state_hashed( S );
    if ( !start ) {
      printf("\n\ncurrent start state not hashed! debug me!\n\n");
      exit( 1 );
    }
    if ( start->step == -1 ) {
      printf("\n\ncurrent start state marked removed from plan! debug me!\n\n");
      exit( 1 );
    }
  }

  for ( j = num_ops - 1; j > -1; j-- ) {
    if ( gnum_plan_ops == MAX_PLAN_LENGTH ) {
      printf("\nincrease MAX_PLAN_LENGTH! currently %d\n\n",
	     MAX_PLAN_LENGTH);
      exit( 1 );
    }
    start->next_step = hash_plan_state( states[j], gnum_plan_ops + 1 );
    start = start->next_step;
    source_to_dest( &(gplan_states[gnum_plan_ops+1]), states[j] );
    gplan_ops[gnum_plan_ops++] = ops[j];
  }

}




PlanHashEntry *hash_plan_state( State *S, int step )

{

  int sum, index;
  PlanHashEntry *h, *tmp;

  sum = state_sum( S );
  index = sum & PLAN_HASH_BITS;

  for ( h = lplan_hash_entry[index]; h; h = h->next ) {
    if ( h->sum != sum ) continue;
    if ( same_state( S, &(h->S) ) ) break;
  }

  if ( h ) {
    if ( h->step != -1 ) {
      printf("\n\nreencountering a state that is already in plan! debug me\n\n");
      exit( 1 );
    }
    h->step = step;
    return h;
  }

  for ( h = lplan_hash_entry[index]; h && h->next; h = h->next );

  tmp = new_PlanHashEntry();
  tmp->sum = sum;
  copy_source_to_dest( &(tmp->S), S );
  tmp->step = step;

  if ( h ) {
    h->next = tmp;
  } else {
    lplan_hash_entry[index] = tmp;
  }

  return tmp;

}
  

 
PlanHashEntry *plan_state_hashed( State *S )

{

  int sum, index;
  PlanHashEntry *h;

  sum = state_sum( S );
  index = sum & PLAN_HASH_BITS;

  for ( h = lplan_hash_entry[index]; h; h = h->next ) {
    if ( h->sum != sum ) continue;
    if ( same_state( S, &(h->S) ) ) break;
  }

  if ( h && h->step != -1 ) {
    return h;
  }

  return NULL;

}














/*********************************
 * ADDED GOAL DELETION HEURISTIC *
 *********************************/













Bool new_goal_gets_deleted( EhcNode *n )

{

  int i, j, ef, new_goal = n->new_goal;

  for ( i = 0; i < gnum_in_plan_E; i++ ) {
    ef = gin_plan_E[i];
    for ( j = 0; j < gef_conn[ef].num_D; j++ ) {
      if ( gef_conn[ef].D[j] == new_goal ) {
	return TRUE;
      }
    }
  }

  return FALSE;

}















/************************************
 * BEST FIRST SEARCH IMPLEMENTATION *
 ************************************/














Bool do_best_first_search( void )

{

  static Bool fc = TRUE;
  static State S;

  BfsNode *first;
  int i, min = INFINITY;
  Bool start = TRUE;

  if ( fc ) {
    make_state( &S, gnum_ft_conn );
    S.max_F = gnum_ft_conn;
    fc = FALSE;
  }

  lbfs_space_head = new_BfsNode();
  lbfs_space_had = NULL;

  for ( i = 0; i < BFS_HASH_SIZE; i++ ) {
    lbfs_hash_entry[i] = NULL;
  }

  add_to_bfs_space( &ginitial_state, -1, NULL );

  while ( TRUE ) {
    if ( (first = lbfs_space_head->next) == NULL ) {
      printf("\n\nbest first search space empty! problem proven unsolvable.\n\n");
      return FALSE;
    }

    lbfs_space_head->next = first->next;
    if ( first->next ) {
      first->next->prev = lbfs_space_head;
    }

    if ( LESS( first->h, min ) ) {
      min = first->h;
      if ( start ) {
	printf("\nadvancing to distance : %4d", min);
	start = FALSE;
      } else {
	printf("\n                        %4d", min);
      }
    }

    if ( first->h == 0 ) {
      break;
    }

    get_A( &(first->S) );
    for ( i = 0; i < gnum_A; i++ ) {
      result_to_dest( &S, &(first->S), gA[i] );
      add_to_bfs_space( &S, gA[i], first );
    }

    first->next = lbfs_space_had;
    lbfs_space_had = first;
  }

  extract_plan( first );
  return TRUE;

}



void add_to_bfs_space( State *S, int op, BfsNode *father )

{

  BfsNode *new, *i;
  int h;

  /* see if state is already a part of this search space
   */
  if ( bfs_state_hashed( S ) ) {
    return;
  }

  h = get_1P( S, &ggoal_state );

  if ( h == INFINITY ) {
    return;
  }

  for ( i = lbfs_space_head; i->next; i = i->next ) {
    if ( i->next->h > h ) break;
  }

  new = new_BfsNode();
  copy_source_to_dest( &(new->S), S );
  new->op = op;
  new->h = h;
  new->father = father;
  
  new->next = i->next;
  new->prev = i;
  i->next = new;
  if ( new->next ) {
    new->next->prev = new;
  }
  
  hash_bfs_node( new );

}



void extract_plan( BfsNode *last )

{

  BfsNode *i;
  int ops[MAX_PLAN_LENGTH], num_ops;
  State_pointer states[MAX_PLAN_LENGTH];
  int j;

  num_ops = 0;
  for ( i = last; i->op != -1; i = i->father ) {
    if ( num_ops == MAX_PLAN_LENGTH ) {
      printf("\nincrease MAX_PLAN_LENGTH! currently %d\n\n",
	     MAX_PLAN_LENGTH);
      exit( 1 );
    }
    states[num_ops] = &(i->S);
    ops[num_ops++] = i->op;
  }

  gnum_plan_ops = 0;
  for ( j = num_ops - 1; j > -1; j-- ) {
    source_to_dest( &(gplan_states[gnum_plan_ops+1]), states[j] );
    gplan_ops[gnum_plan_ops++] = ops[j];
  }

}












/************************************************************
 * HASHING ALGORITHM FOR RECOGNIZING REPEATED STATES IN BFS *
 ************************************************************/












void hash_bfs_node( BfsNode *n )

{

  int sum, index;
  BfsHashEntry *h, *tmp;

  sum = state_sum( &(n->S) );
  index = sum & BFS_HASH_BITS;

  h = lbfs_hash_entry[index];
  if ( !h ) {
    h = new_BfsHashEntry();
    h->sum = sum;
    h->bfs_node = n;
    lbfs_hash_entry[index] = h;
    return;
  }
  for ( ; h->next; h = h->next );

  tmp = new_BfsHashEntry();
  tmp->sum = sum;
  tmp->bfs_node = n;
  h->next = tmp;
      
}



Bool bfs_state_hashed( State *S )

{

  int sum, index;
  BfsHashEntry *h;

  sum = state_sum( S );
  index = sum & BFS_HASH_BITS;

  h = lbfs_hash_entry[index];
  for ( h = lbfs_hash_entry[index]; h; h = h->next ) {
    if ( h->sum != sum ) {
      continue;
    }
    if ( same_state( &(h->bfs_node->S), S ) ) {
      return TRUE;
    }
  }

  return FALSE;

}












/****************************
 * STATE HANDLING FUNCTIONS *
 ****************************/












/* function that computes state transition as induced by a
 * normalized ADL action. Adds go before deletes!
 *
 * a bit odd in implementation:
 * function returns number of new goal that came in when applying
 * op to source; needed for Goal Added Deletion Heuristic
 */
int result_to_dest( State *dest, State *source, int op )

{

  static Bool first_call = TRUE;
  static Bool *in_source, *in_dest, *in_del, *true_ef;
  static int *del, num_del;

  int i, j, ef;
  int r = -1;
  
  if ( first_call ) {
    in_source = ( Bool * ) calloc( gnum_ft_conn, sizeof( Bool ) );
    in_dest = ( Bool * ) calloc( gnum_ft_conn, sizeof( Bool ) );
    in_del = ( Bool * ) calloc( gnum_ft_conn, sizeof( Bool ) );
    true_ef = ( Bool * ) calloc( gnum_ef_conn, sizeof( Bool ) );
    del = ( int * ) calloc( gnum_ft_conn, sizeof( int ) );
    for ( i = 0; i < gnum_ft_conn; i++ ) {
      in_source[i] = FALSE;
      in_dest[i] = FALSE;
      in_del[i] = FALSE;
    }
    for ( i = 0; i < gnum_ef_conn; i++ ) {
      true_ef[i] = FALSE;
    }
    first_call = FALSE;
  }

  /* setup true facts for effect cond evaluation
   */
  for ( i = 0; i < source->num_F; i++ ) {
    in_source[source->F[i]] = TRUE;
  }

  /* setup deleted facts
   */
  num_del = 0;
  for ( i = 0; i < gop_conn[op].num_E; i++ ) {
    ef = gop_conn[op].E[i];
    for ( j = 0; j < gef_conn[ef].num_PC; j++ ) {
      if ( !in_source[gef_conn[ef].PC[j]] ) break;
    }
    if ( j < gef_conn[ef].num_PC ) continue;
    true_ef[i] = TRUE;
    for ( j = 0; j < gef_conn[ef].num_D; j++ ) {
      if ( in_del[gef_conn[ef].D[j]] ) continue;
      in_del[gef_conn[ef].D[j]] = TRUE;
      del[num_del++] = gef_conn[ef].D[j];
    }
  }

  /* put all non-deleted facts from source into dest.
   * need not check for put-in facts here,
   * as initial state is made doubles-free, and invariant keeps
   * true through the transition procedure
   */
  dest->num_F = 0;
  for ( i = 0; i < source->num_F; i++ ) {
    if ( in_del[source->F[i]] ) {
      continue;
    }
    dest->F[dest->num_F++] = source->F[i];
    in_dest[source->F[i]] = TRUE;
  }

  /* now, finally, add all fullfilled effect adds to dest; 
   * each fact at most once!
   */
  for ( i = 0; i < gop_conn[op].num_E; i++ ) {
    if ( !true_ef[i] ) continue;
    ef = gop_conn[op].E[i];
    for ( j = 0; j < gef_conn[ef].num_A; j++ ) {
      if ( in_dest[gef_conn[ef].A[j]] ) {
	continue;
      }
      dest->F[dest->num_F++] = gef_conn[ef].A[j];
      in_dest[gef_conn[ef].A[j]] = TRUE;
      if ( gft_conn[gef_conn[ef].A[j]].is_global_goal ) {
	r = gef_conn[ef].A[j];
      }
    }
  }

  /* unset infos
   */
  for ( i = 0; i < source->num_F; i++ ) {
    in_source[source->F[i]] = FALSE;
  }
  for ( i = 0; i < dest->num_F; i++ ) {
    in_dest[dest->F[i]] = FALSE;
  }
  for ( i = 0; i < num_del; i++ ) {
    in_del[del[i]] = FALSE;
  }
  for ( i = 0; i < gop_conn[op].num_E; i++ ) {
    true_ef[i] = FALSE;
  }

  return r;

}



void source_to_dest( State *dest, State *source )

{

  int i;

  for ( i = 0; i < source->num_F; i++ ) {
    dest->F[i] = source->F[i];
  }
  dest->num_F = source->num_F;

}



void copy_source_to_dest( State *dest, State *source )

{

  int i, m;

  if ( dest->max_F < source->num_F ) {
    if ( dest->F ) {
      free( dest->F );
    }
    if ( source->num_F + 50 > gnum_ft_conn ) {
      m = gnum_ft_conn;
    } else {
      m = source->num_F + 50;
    }
    dest->F = ( int * ) calloc( m, sizeof( int ) );
    dest->max_F = m;
  }

  for ( i = 0; i < source->num_F; i++ ) {
    dest->F[i] = source->F[i];
  }
  dest->num_F = source->num_F;

}



void print_state( State S )

{

  int i;
  
  for ( i = 0; i < S.num_F; i++ ) {
    printf("\n");
    print_ft_name( S.F[i] );
  }

}
