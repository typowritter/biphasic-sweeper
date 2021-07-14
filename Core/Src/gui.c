/**
  ******************************************************************************
  * @file    gui.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-14
  * @brief   simple event-driven gui
  ******************************************************************************
  */

#include "gui.h"

gui_obj_t gui[] = {
  /* button 1 */
  [0] = {
    .dim     = {.x = 10, .y = 10, .w = 60, .h = 40},
    .class   = CLASS_BUTTON,
    .visible = true,
  }
};


void gui_init()
{

}