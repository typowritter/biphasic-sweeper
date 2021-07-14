/**
  ******************************************************************************
  * @file    gui.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-14
  * @brief   simple event-driven gui
  ******************************************************************************
  */

#ifndef __GUI_H
#define __GUI_H

#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

#include "lcd.h"

#define TEXT_BUFLEN   20
/**
 * Coordinate definition.
 */
typedef struct
{
  uint16_t x,
  uint16_t y,
  uint16_t w,
  uint16_t h,
} gui_area_t;

/**
 * Type of event being sent to the object.
 */
typedef enum {
  EVENT_PRESSED,        /**< The object has been pressed*/
  EVENT_CLICKED,        /**< Called on release */
  EVENT_PRESS_LOST,     /**< The object is still being pressed but slid finger off of the object */
  EVENT_VALUE_CHANGED,  /**< The object's value has changed (i.e. textbox) */
  EVENT_INSERT,         /**< A text is inserted to the object. The event data is `char *` being inserted. */
} event_code_t;

typedef struct
{
  struct gui_obj_t *target;
  event_code_t code;
  void *user_data;
} gui_event_t;

/**
 * Widget object definition
 */
typedef enum {
  CLASS_LINE,
  CLASS_LABEL,
  CLASS_BUTTON,
  CLASS_TEXTBOX,
  CLASS_KEYPAD,
  CLASS_BITMAP,
} widget_class_t;

typedef struct
{
  color_t color;
} line_t;

typedef struct
{
  color_t fg;
  char text[TEXT_BUFLEN];
} label_t;

typedef struct
{
  color_t bg;
  char prompt[10];
  void (*event_cb)(gui_event_t *e);
} button_t;

typedef struct
{
  int cursor_pos;
  int len;
  char buf[TEXT_BUFLEN];
} textbox_t;

typedef struct
{
  textbox_t input;
  button_t grid[12];
  void (*event_cb)(gui_event_t *e);
} keypad_t;

typedef struct
{
  void (*event_cb)(gui_event_t *e);
} bitmap_t;

typedef struct
{
  gui_area_t dim;
  widget_class_t class;
  bool visible;
  void* generic_widget;
} gui_obj_t;

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __GUI_H */
