/*
	 Copyright (C) 2011 Georgia Institute of Technology, University of Utah, Weill Cornell Medical College

	 This program is free software: you can redistribute it and/or modify
	 it under the terms of the GNU General Public License as published by
	 the Free Software Foundation, either version 3 of the License, or
	 (at your option) any later version.

	 This program is distributed in the hope that it will be useful,
	 but WITHOUT ANY WARRANTY; without even the implied warranty of
	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	 GNU General Public License for more details.

	 You should have received a copy of the GNU General Public License
	 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef SCOPE_H
#define SCOPE_H

#include <QEvent>
#include <QPen>
#include <QPixmap>
#include <QtGui>
#include <QWidget>

#include <qwt_scale_engine.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_directpainter.h>
#include <qwt_plot.h>
#include <qwt.h>
#include <qwt_curve_fitter.h>
#include <qwt_painter.h>
#include <qwt_system_clock.h>
#include <qwt_interval.h>

#include <list>
#include <vector>

class QwtPlotCurve;
class QwtPlotMarker;
class QwtPlotDirectPainter;

class Scope : public QwtPlot {

	Q_OBJECT

	public:

		class Channel {
			friend class Scope;

			public:
			Channel(void);
			virtual ~Channel(void);
			void *getInfo(void);
			const void *getInfo(void) const;
			double getScale(void) const;
			double getOffset(void) const;
			QPen getPen(void) const;
			QString getLabel(void) const;

			private:
			QPen pen;
			QString label;
			double scale;
			double offset;
			std::vector<double> prevdata;
			std::vector<double> data;
			void *info;
		}; // class Channel

		class Canvas : public QwtPlotCanvas {

			public:
				Canvas(QwtPlot *plot = NULL) : QwtPlotCanvas(plot) {

					// The backing store is important, when working with widget
					// overlays ( f.e rubberbands for zooming ).
					// Here we don't have them and the internal
					// backing store of QWidget is good enough.
					setPaintAttribute(QwtPlotCanvas::BackingStore, false);

					if(QwtPainter::isX11GraphicsSystem()) {
#if QT_VERSION < 0x050000
						// Even if not liked by the Qt development, Qt::WA_PaintOutsidePaintEvent
						// works on X11. This has a nice effect on the performance.
						setAttribute( Qt::WA_PaintOutsidePaintEvent, true );
#endif

						// Disabling the backing store of Qt improves the performance
						// for the direct painter even more, but the canvas becomes
						// a native window of the window system, receiving paint events
						// for resize and expose operations. Those might be expensive
						// when there are many points and the backing store of
						// the canvas is disabled. So in this application
						// we better don't both backing stores.
						if(testPaintAttribute(QwtPlotCanvas::BackingStore)) {
							setAttribute(Qt::WA_PaintOnScreen, true);
							setAttribute(Qt::WA_NoSystemBackground, true);
						}
					}
					setupPalette();
				}

			private:
				void setupPalette()	{
					QPalette pal = palette();

#if QT_VERSION >= 0x040400
					QLinearGradient gradient;
					gradient.setCoordinateMode(QGradient::StretchToDeviceMode);
					gradient.setColorAt(1.0, QColor(Qt::white));
					pal.setBrush(QPalette::Window, QBrush(gradient));
#else
					pal.setBrush(QPalette::Window, QBrush(color));
#endif
					pal.setColor(QPalette::WindowText, Qt::green);
					setPalette(pal);
				}
		}; // class Canvas

		enum trig_t{
			NONE,
			POS,
			NEG,
		};

		Scope(QWidget * = NULL);
		virtual ~Scope(void);

		bool paused(void) const;
		std::list<Channel>::iterator insertChannel(QString,double,double,const QPen &,void *);
		void *removeChannel(std::list<Channel>::iterator);
		size_t getChannelCount(void) const;
		std::list<Channel>::iterator getChannelsBegin(void);
		std::list<Channel>::iterator getChannelsEnd(void);

		std::list<Channel>::const_iterator getChannelsBegin(void) const;
		std::list<Channel>::const_iterator getChannelsEnd(void) const;

		void clearData(void);
		void setData(double *,size_t);
		size_t getDataSize(void) const;
		void setDataSize(size_t);

		trig_t getTriggerDirection(void);
		double getTriggerThreshold(void);
		std::list<Channel>::iterator getTriggerChannel(void);
		bool getTriggerHolding(void);
		double getTriggerHoldoff(void);
		void setTrigger(trig_t,double,std::list<Channel>::iterator,bool,double);

		double getDivT(void) const;
		void setDivT(double);

		void setPeriod(double);
		size_t getDivX(void) const;
		size_t getDivY(void) const;
		//void setDivXY(size_t,size_t);

		size_t getRefresh(void) const;
		void setRefresh(size_t);

		void setChannelScale(std::list<Channel>::iterator,double);
		void setChannelOffset(std::list<Channel>::iterator,double);
		void setChannelPen(std::list<Channel>::iterator,const QPen &);
		void setChannelLabel(std::list<Channel>::iterator,const QString &);

		public slots:
		void timeoutEvent(void);
		void togglePause(void);

	protected:
		//void paintEvent(QPaintEvent *);
		//void resizeEvent(QResizeEvent *);

	private:
		void drawBackground(void);
		//QRect drawForeground(void);

		void positionLabels(QPainter &);
		//void refreshBackground(void);

		bool drawZero;
		size_t divX;
		size_t divY;
		size_t data_idx;
		size_t data_size;
		double hScl;        // horizontal scale for time (ms)
		double period;      // real-time period of system (ms)
		size_t refresh;

		bool triggering;
		bool triggerHolding;
		trig_t triggerDirection;
		double triggerThreshold;
		double triggerHoldoff;
		std::list<size_t> triggerQueue;
		std::list<Channel>::iterator triggerChannel;
		size_t triggerLast;

		// Scope primary paint element
		QwtPlotDirectPainter *d_directPainter;

		// Scope painter elements
		QwtPlotMarker *d_origin;
		QwtPlotCurve *d_curve;
		int d_paintedPoints;

		bool isPaused;
		//QPixmap background;
		QPixmap foreground;
		QRect drawRect;
		QTimer *timer;
		QString dtLabel;
		std::list<Channel> channels;
}; // class Scope

#endif // SCOPE_H
