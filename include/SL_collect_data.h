/*!=============================================================================
  ==============================================================================

  \file    SL_collect_data.h

  \author  Stefan Schaal
  \date    Feb 1999

  ==============================================================================
  \remarks
  
  collect_data specific defines
  
  ============================================================================*/
  
#ifndef _SL_collect_data_
#define _SL_collect_data_

/* possible data types */

#define FLOAT  1
#define DOUBLE 2
#define INT    3
#define SHORT  4
#define LONG   5
#define CHAR   6

#ifdef __cplusplus
extern "C" {
#endif
  
  /* shared functions */
  
  void addVarToCollect(char *ptr,char *name,char *units, int type, int flag);
  void initCollectData( int freq );
  void writeToBuffer(void);
  void saveData(void);
  void scds(void);
  void mscds(int num);
  void scd(void);
  void stopcd(void);
  int  dscd(int parm);
  int  startCollectData(void);
  void outMenu(void);
  void changeCollectFreq( int freq );
  void updateDataCollectScript( void );
  int  getDataCollectPtr( char *name, int n_char_units, void **vptr, int *type, char *units );
  
  
#ifdef __cplusplus
}
#endif

#endif  /* _SL_collect_data_ */
