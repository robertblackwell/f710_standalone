#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <string>
#include <vector>
#include <sys/eventfd.h>
#include <mutex>
#include "background_thread.h"
#include "app_context.h"

AppContext app_context;
AppContext* app_context_ptr = &app_context;

std::string te_text{};
void te_save_text()
{
    auto saved_txt = gtk_entry_buffer_get_text(app_context.teb);
    te_text = std::string(saved_txt);
}
void te_restore_text()
{
    gtk_entry_buffer_set_text(app_context.teb, te_text.c_str(), -1);
}


int msg_count = 0;
bool history_active = false;
uint history_index = 0;
std::vector<std::string*> history_cmds;
void history_display_cmd()
{
    if(history_active) {
        auto teb = app_context.teb;
        auto s = *history_cmds[history_index];
        gtk_entry_buffer_set_text(teb, s.c_str(), -1);
    }
}
void up_arrow_pressed()
{
}
void down_arrow_pressed()
{

}
void up_arrow_released()
{
    if(history_cmds.empty()) {
        return;
    }
    if(!history_active) {
        history_index = history_cmds.size()-1;
        history_active = true;
    } else {
        if(history_index > 0) {
            history_index--;
        }
    }
    history_display_cmd();
}
void down_arrow_released()
{
    printf("downarrow released\n");
    if(history_cmds.empty()) {
        return;
    }
    if(history_active) {
        if(history_index >= history_cmds.size() - 1) {
            ;
        } else {
            history_index++;
            history_display_cmd();
        }
    } else {
        printf("down arrow history not active\n");
    }
}

gboolean event_key_pressed_cb(
        GtkEventController* controller,
        guint      keyval,
        guint      keycode,
        GdkModifierType state,
        GtkWidget* widget
        )
{
    printf("keypressed cb widget: %p keyval: %d keycode: %d state: %x  controller:%p\n",(void*)widget, keyval, keycode, state, (void*)controller);
    switch(keycode) {
        case 111:
            up_arrow_pressed();
            break;
        case 116:
            down_arrow_pressed();
            break;
        default:
            break;
    }
    return TRUE;
}
gboolean event_key_released_cb(
        GtkEventController* controller,
        guint      keyval,
        guint      keycode,
        GdkModifierType state,
        GtkWidget* widget
)
{
    printf("key released cb widget: %p keyval: %d keycode: %d state: %x  controller:%p\n", (void*)widget, keyval, keycode, state, (void*)controller);
    switch(keycode) {
        case 111:
            up_arrow_released();
            break;
        case 116:
            down_arrow_released();
            break;
        default:
            break;
    }
    return TRUE;
}
void set_focus_te()
{
    gtk_window_set_focus(reinterpret_cast<GtkWindow *>(app_context.win), app_context.te);
    gtk_widget_grab_focus(app_context.te);
}
void entry_activate(void* te)
{
    auto teb = gtk_entry_get_buffer((GtkEntry*)te);
    auto s = gtk_entry_buffer_get_text(teb);
    // handle history
    history_active = false;
    auto cmd = new std::string(s);
    history_cmds.push_back(cmd);

    // build io buffer for send_threadsafe
    char* bptr;
    auto len = asprintf(&bptr, "%s\n", s);
    auto iob_uptr = std::make_unique<IoBuffer>();
    iob_uptr->append(bptr);
    free(bptr);

	g_print("entry activate %s\n", s);
    gtk_entry_buffer_set_text(teb, "", 0);
    set_focus_te();

    background_send_threadsafe(std::move(iob_uptr));


	// auto x =  g_io_add_watch;
}
void add_text_to_tv(std::string* msg)
{
    GtkTextIter iter;
    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);
    gtk_text_buffer_insert(app_context.tvb, &iter, "\n", -1);
    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);
    gtk_text_buffer_insert(app_context.tvb, &iter, msg->c_str(), -1);
//    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);
//    gtk_text_buffer_insert(app_context.tvb, &iter, "\n", -1);
//    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);

    gtk_text_view_scroll_mark_onscreen(reinterpret_cast<GtkTextView *>(app_context.tv), app_context.tv_end_mark);
}
void add_c_str_to_tv(const char* c_str)
{
    auto st = new std::string(c_str);
    add_text_to_tv(st);
    delete st;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
gboolean main_thread_on_message(GIOChannel* source, GIOCondition condition, gpointer data )
{
#pragma GCC diagnostic pop
    pthread_t id = pthread_self();
    printf("message callback id: %ld\n", id);
    auto bc = (BackgroundContext*)data;
    auto msg = background_get_message(bc);
    printf("message callback source:%p msg_ptr:%p msg:%s  \n", (void*)source, (void*)msg, msg->c_str());
#if 0
    auto msg = q_get();
    printf("message callback  msg_ptr:%p msg:%s  \n", msg, msg->c_str());
    char buf[10];
    long n = read(wa->fd_read, buf, 1);
#endif
    add_text_to_tv(msg);
//    set_focus_te();
#if 0
//    gtk_text_buffer_set_text (tvb, msg->c_str(), -1);
    GtkTextIter iter;
    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);
    gtk_text_buffer_insert(app_context.tvb, &iter, msg->c_str(), -1);
    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);
    gtk_text_buffer_insert(app_context.tvb, &iter, "\n", -1);
    gtk_text_buffer_get_end_iter(app_context.tvb, &iter);

    gtk_text_view_scroll_mark_onscreen(reinterpret_cast<GtkTextView *>(app_context.tv), app_context.tv_end_mark);
#endif
    delete msg;
    return TRUE;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void app_activate (GApplication *app, void* arg)
{
#pragma GCC diagnostic pop

    printf("app_activate %p\n", (void*)&app_context);

    background_init();

    app_context.win = gtk_application_window_new (GTK_APPLICATION (app));
	gtk_window_set_title (GTK_WINDOW (app_context.win), "Pico Serial Monitor");
	gtk_window_set_default_size (GTK_WINDOW (app_context.win), 900, 1300);

    app_context.main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);

    // Command entry line
    app_context.te = gtk_entry_new();
    app_context.teb = gtk_entry_get_buffer((GtkEntry*)app_context.te);
	g_signal_connect(app_context.te, "activate", G_CALLBACK(entry_activate), app_context.te);

    // Text view must be wrapped in a scrolled window
    app_context.scroll_window = gtk_scrolled_window_new();
    gtk_scrolled_window_set_propagate_natural_height((GtkScrolledWindow*)app_context.scroll_window, TRUE);

    app_context.tv = gtk_text_view_new ();
    gtk_widget_set_focusable(app_context.tv, false);

    gtk_text_view_set_top_margin((GtkTextView*)app_context.tv, 10);
    gtk_text_view_set_right_margin((GtkTextView*)app_context.tv, 10);
    gtk_text_view_set_left_margin((GtkTextView*)app_context.tv, 10);
    app_context.tvb = gtk_text_view_get_buffer (GTK_TEXT_VIEW (app_context.tv));

    // put a right-gravity mark that stays at the end of the text even after modifications
    app_context.tv_end_mark = gtk_text_mark_new("LastLine", FALSE);
    GtkTextIter end_iter;
    gtk_text_buffer_get_end_iter(app_context.tvb, &end_iter);
    gtk_text_buffer_add_mark((GtkTextBuffer*)app_context.tvb, app_context.tv_end_mark, &end_iter);

//	gtk_text_buffer_set_text (app_context.tvb, text, -1);
	gtk_text_view_set_wrap_mode (GTK_TEXT_VIEW (app_context.tv), GTK_WRAP_WORD_CHAR);
    gtk_scrolled_window_set_child((GtkScrolledWindow*)app_context.scroll_window, app_context.tv);

	gtk_box_append((GtkBox*) app_context.main_box, app_context.te);
	gtk_box_append((GtkBox*) app_context.main_box, app_context.scroll_window);
	gtk_window_set_child (GTK_WINDOW (app_context.win), app_context.main_box);

    app_context.te_key_event_controller = gtk_event_controller_key_new();
    g_signal_connect_object(app_context.te_key_event_controller, //.te_key_event_controller,
                            "key_pressed",
                            G_CALLBACK(event_key_pressed_cb),
                            app_context.win,
                            (GConnectFlags)0);
    g_signal_connect_object(app_context.te_key_event_controller, //.te_key_event_controller,
                            "key_released",
                            G_CALLBACK(event_key_released_cb),
                            app_context.win,
                            (GConnectFlags)0);

    gtk_widget_add_controller(GTK_WIDGET(app_context.te), app_context.te_key_event_controller);

	gtk_window_present (GTK_WINDOW (app_context.win));

    background_start_thread(main_thread_on_message);
}
int main (int argc, char **argv) {
    GtkApplication *app;
    int stat;

    app = gtk_application_new ("com.github.ToshioCP.tfv1", G_APPLICATION_FLAGS_NONE);
    pthread_t id = pthread_self();
    printf("main app_content %p  main thread id: %ld\n", (void*)&app_context, id);
    g_signal_connect (app, "activate", G_CALLBACK (app_activate), (&app_context));
    stat = g_application_run (G_APPLICATION (app), argc, argv);
    background_join();
//    auto wa = get_background_context();
//    pthread_join(wa->background_thread, nullptr);
    g_object_unref (app);
    return stat;
}