#include "flock_simulation/flock.h"
#include <cairo.h>
#include <gtk/gtk.h>
#include <iostream>
#include <math.h>
#include <ostream>

static void do_drawing(cairo_t*, GtkWidget*);

static FlockSimulation::Flock flock(10, 10, 10);

static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr,
    gpointer user_data)
{
  do_drawing(cr, widget);

  return FALSE;
}

static void do_drawing(cairo_t *cr, GtkWidget *widget)
{
    std::cout << "draw" << std::endl;

    GtkWidget* win = gtk_widget_get_toplevel(widget);
    int width, height;
    gtk_window_get_size(GTK_WINDOW(win), &width, &height);

    cairo_set_line_width(cr, 9);
    cairo_set_source_rgb(cr, 0.69, 0.19, 0);

    cairo_translate(cr, width / 2.0, height / 2.0);
    cairo_arc(cr, 0, 0, 50, 0, 2 * M_PI);
    cairo_stroke_preserve(cr);

    cairo_set_source_rgb(cr, 0.3, 0.4, 0.6);
    cairo_fill(cr);
}

const int REFRESH_INTERVAL_REDRAW = 100;

static void sendRedrawSignals(GtkWidget *widget) {
  gtk_widget_queue_draw(widget);
}

int main (int argc, char *argv[])
{
  GtkWidget *window;
  GtkWidget *darea;
  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  darea = gtk_drawing_area_new();
  gtk_container_add(GTK_CONTAINER(window), darea);
  g_signal_connect(G_OBJECT(darea), "draw",
      G_CALLBACK(on_draw_event), NULL);
  g_signal_connect(G_OBJECT(window), "destroy",
      G_CALLBACK(gtk_main_quit), NULL);

  g_timeout_add(REFRESH_INTERVAL_REDRAW, (GSourceFunc) sendRedrawSignals, window);

  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(window), 1024, 600);
  gtk_window_set_title(GTK_WINDOW(window), "Flock simulation");

  gtk_widget_show_all(window);

  gtk_main();

  return 0;
}
