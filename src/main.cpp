#include "flockingbird.h"
#include <cairo.h>
#include <gtk/gtk.h>
#include <iostream>
#include <math.h>
#include <ostream>

static void do_drawing(cairo_t*, GtkWidget*);

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 600;
const int REFRESH_INTERVAL_REDRAW = 30;

double speedLimit = 3;
double forceLimit = 0.03;
double positionIncrementScalingFactor = 1;
double avoidanceRadius = 25;
double visionRange = 50;
double separationWeight = 1.5;
double alignmentWeight = 1.0;
double cohesionWeight = 1.0;

static FlockSimulationParameters flockSimulationParameters(speedLimit,
                                                           forceLimit,
                                                           positionIncrementScalingFactor,
                                                           avoidanceRadius,
                                                           visionRange,
                                                           separationWeight,
                                                           alignmentWeight,
                                                           cohesionWeight);
static Flock flock(10, 10, 10);
static FlockSimulation flockSimulation(flockSimulationParameters, flock, defaultRules);
static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr,
    gpointer user_data)
{
  do_drawing(cr, widget);
  return FALSE;
}

inline double wrap(double val, double max) {
    return val - max * floor(val / max);
}

static void do_drawing(cairo_t *cr, GtkWidget *widget)
{
    const double dotSize = 10;
    // std::cout << "draw" << std::endl;
    cairo_set_source_rgb(cr, 0.69, 0.19, 0);
    for (auto it = flock.boids.begin(); it != flock.boids.end(); it ++) {
      double x = wrap(it->position.x, SCREEN_WIDTH);
      double y = wrap(it->position.y, SCREEN_HEIGHT);
      cairo_set_line_width(cr, dotSize);
      cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND); /* Round dot*/
      cairo_move_to(cr, x, y);
      cairo_line_to(cr, x, y); /* a very short line is a dot */
      cairo_stroke(cr);
    }
    cairo_fill(cr);
}


static void sendRedrawSignals(GtkWidget *widget) {
  gtk_widget_queue_draw(widget);
  flockSimulation.step();
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
  gtk_window_set_default_size(GTK_WINDOW(window), SCREEN_WIDTH, SCREEN_HEIGHT);
  gtk_window_set_title(GTK_WINDOW(window), "Flock simulation");

  gtk_widget_show_all(window);

  gtk_main();

  return 0;
}
