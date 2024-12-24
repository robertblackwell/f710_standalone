//
// Created by robert on 12/22/24.
//

#ifndef GTKTEST_APP_CONTEXT_H
#define GTKTEST_APP_CONTEXT_H
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <string>
#include <sys/eventfd.h>
#include <mutex>
#include <thread>

struct AppContext {
//    GtkWindow *win;
    GtkWidget *win;
    GtkWidget* main_box;
    GtkWidget*     te;
    GtkEntryBuffer* teb;
    GtkWidget*     scroll_window;
    GtkWidget*     tv;
    GtkTextBuffer* tvb;
    GtkTextMark* tv_end_mark;
    GIOChannel* worker_channel;
    // event controller for te key press/release
    GtkEventController* te_key_event_controller;

    AppContext()
    {
        win = nullptr;
        main_box = nullptr;
        te = nullptr;
        teb = nullptr;
        scroll_window =  nullptr;
        tv = nullptr;
        tvb = nullptr;
        tv_end_mark = nullptr;
        worker_channel = nullptr;
    }

};

#endif //GTKTEST_APP_CONTEXT_H
