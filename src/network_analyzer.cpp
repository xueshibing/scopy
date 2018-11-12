/*
 * Copyright 2016 Analog Devices, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file LICENSE.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "logging_categories.h"
#include "dynamicWidget.hpp"
#include "network_analyzer.hpp"
#include "signal_generator.hpp"
#include "spinbox_a.hpp"
#include "osc_adc.h"
#include "hardware_trigger.hpp"
#include "ui_network_analyzer.h"
#include "filemanager.h"

#include <gnuradio/analog/sig_source.h>
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/complex_to_arg.h>
#include <gnuradio/blocks/complex_to_mag_squared.h>
#include <gnuradio/blocks/float_to_short.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/moving_average.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/multiply_conjugate_cc.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/null_source.h>
#include <gnuradio/blocks/rotator_cc.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/blocks/vector_sink.h>

#include <boost/make_shared.hpp>

#include <QDebug>
#include <QThread>
#include <QFileDialog>
#include <QDateTime>
#include <QElapsedTimer>

#include <iio.h>
#include <network_analyzer_api.hpp>

/* This should go away ASAP... */
#define DAC_BIT_COUNT   12
#define INTERP_BY_100_CORR 1.168 // correction value at an interpolation by 100
#define AMPLITUDE_VOLTS	5.0

using namespace adiscope;
using namespace gr;

NetworkAnalyzer::NetworkAnalyzer(struct iio_context *ctx, Filter *filt,
		std::shared_ptr<GenericAdc> &adc_dev,
		QPushButton *runButton, QJSEngine *engine,
		ToolLauncher *parent) :
	Tool(ctx, runButton, new NetworkAnalyzer_API(this), "Network Analyzer", parent),
	ui(new Ui::NetworkAnalyzer),
	adc_dev(adc_dev),
	d_cursorsEnabled(false),
	stop(true), amp1(nullptr), amp2(nullptr),
	wheelEventGuard(nullptr)
{
	iio = iio_manager::get_instance(ctx,
			filt->device_name(TOOL_NETWORK_ANALYZER, 2));

	adc = filt->find_device(ctx, TOOL_NETWORK_ANALYZER, 2);
	dac1 = filt->find_channel(ctx, TOOL_NETWORK_ANALYZER, 0, true);
	dac2 = filt->find_channel(ctx, TOOL_NETWORK_ANALYZER, 1, true);
	if (!dac1 || !dac2)
		throw std::runtime_error("Unable to find channels in filter file");

	/* FIXME: TODO: Move this into a HW class / lib M2k */
	struct iio_device *fabric = iio_context_find_device(ctx, "m2k-fabric");
	if (fabric) {
		this->amp1 = iio_device_find_channel(fabric, "voltage0", true);
		this->amp2 = iio_device_find_channel(fabric, "voltage1", true);
		if (amp1 && amp2) {
			iio_channel_attr_write_bool(amp1, "powerdown", true);
			iio_channel_attr_write_bool(amp2, "powerdown", true);
		}
	}

	ui->setupUi(this);


	connect(ui->run_button, SIGNAL(toggled(bool)),
			runButton, SLOT(setChecked(bool)));
	connect(runButton, SIGNAL(toggled(bool)),
			ui->run_button, SLOT(setChecked(bool)));
	connect(ui->run_button, SIGNAL(toggled(bool)),
			this, SLOT(startStop(bool)));
	connect(this, &NetworkAnalyzer::sweepDone,
			[=]() {
		ui->run_button->setChecked(false);
	});


	ui->rightMenu->setMaximumWidth(0);

	const struct iio_device *dev1 = iio_channel_get_device(dac1);
	unsigned long max_samplerate1 =
		SignalGenerator::get_max_sample_rate(dev1);

	const struct iio_device *dev2 = iio_channel_get_device(dac2);
	unsigned long max_samplerate2 =
		SignalGenerator::get_max_sample_rate(dev2);

	unsigned long max_samplerate =
		std::min(max_samplerate1, max_samplerate2);

    m_dBgraph.setColor(QColor(255,114,0));
    m_dBgraph.setXTitle("Frequency (Hz)");
    m_dBgraph.setYTitle("Magnitude(dB)");
    m_dBgraph.setXMin(1000.000000);
    m_dBgraph.setXMax(50000.000000);
    m_dBgraph.setYMin(-90.000000);
    m_dBgraph.setYMax(10.000000);
    m_dBgraph.useLogFreq(true);

    m_phaseGraph.setColor(QColor(144,19,254));
    m_phaseGraph.setYTitle("Phase (°)");
    m_phaseGraph.setYUnit("°");
    m_phaseGraph.setXMin(1000.000000);
    m_phaseGraph.setXMax(50000.000000);
    m_phaseGraph.setYMin(-180.000000);
    m_phaseGraph.setYMax(180.000000);
    m_phaseGraph.useLogFreq(true);

    samplesCount = new ScaleSpinButton({
            {"samples",1e0},
    }, "Samples count", 10, 1000, false, false, this);
    samplesCount->setValue(1000);

    minFreq = new ScaleSpinButton({
            {"Hz",1e0},
            {"kHz",1e3},
            {"MHz",1e6}
    },"Min Freq", 1e0, 5e7, false, false, this);
    minFreq->setValue(1000);

    maxFreq = new ScaleSpinButton({
            {"Hz",1e0},
            {"kHz",1e3},
            {"MHz",1e6}
    },"Max Freq", 1e0, 5e7, false, false, this);
    maxFreq->setValue(50000);

    amplitude = new ScaleSpinButton({
            {"μVolts",1e-6},
            {"mVolts",1e-3},
            {"Volts",1e0}
    },"Amplitude", 1e-6, 1e1, false, false, this);
    amplitude->setValue(1);

    offset = new PositionSpinButton({
            {"μVolts",1e-6},
            {"mVolts",1e-3},
            {"Volts",1e0}
    },"Offset", -5, 5, false, false, this);

    offset->setValue(0);

    magMax = new PositionSpinButton({
            {"dB",1e0}
    }, "Max. Magnitude", -120, 120, false, false, this);
    magMax->setValue(10);

    magMin = new PositionSpinButton({
            {"dB",1e0}
    }, "Min. Magnitude", -120, 120, false, false, this);
    magMin->setValue(-90);

    phaseMax = new PositionSpinButton({
            {"dB",1e0}
    }, "Max. Phase", -180, 180, false, false, this);
    phaseMax->setValue(180);

    phaseMin = new PositionSpinButton({
            {"dB",1e0}
    }, "Min. Phase", -180, 180, false, false, this);
    phaseMin->setValue(-180);


    ui->samplesCountLayout->addWidget(samplesCount);
    ui->minFreqLayout->addWidget(minFreq);
    ui->maxFreqLayout->addWidget(maxFreq);
    ui->amplitudeLayout->addWidget(amplitude);
    ui->offsetLayout->addWidget(offset);
    ui->magMaxLayout->addWidget(magMax);
    ui->magMinLayout->addWidget(magMin);
    ui->phaseMaxLayout->addWidget(phaseMax);
    ui->phaseMinLayout->addWidget(phaseMin);

    setMinimumDistanceBetween(magMin, magMax, 1);
    setMinimumDistanceBetween(phaseMin, phaseMax, 1);
    setMinimumDistanceBetween(minFreq, maxFreq, 0);

    connect(magMax, &PositionSpinButton::valueChanged,
            ui->xygraph, &NyquistGraph::setMax);
    connect(magMax, &PositionSpinButton::valueChanged,
            ui->nicholsgraph, &dBgraph::setYMax);
    connect(magMin, &PositionSpinButton::valueChanged,
            ui->xygraph, &NyquistGraph::setMin);
    connect(magMin, &PositionSpinButton::valueChanged,
            ui->nicholsgraph, &dBgraph::setYMin);
    connect(phaseMax, &PositionSpinButton::valueChanged,
            ui->nicholsgraph, &dBgraph::setXMax);
    connect(phaseMin, &PositionSpinButton::valueChanged,
            ui->nicholsgraph, &dBgraph::setXMin);

    connect(minFreq, SIGNAL(valueChanged(double)),
            &m_dBgraph, SLOT(setXMin(double)));
    connect(maxFreq, SIGNAL(valueChanged(double)),
            &m_dBgraph, SLOT(setXMax(double)));
    connect(magMin, SIGNAL(valueChanged(double)),
            &m_dBgraph, SLOT(setYMin(double)));
    connect(magMax, SIGNAL(valueChanged(double)),
            &m_dBgraph, SLOT(setYMax(double)));
    connect(ui->btnIsLog, SIGNAL(toggled(bool)),
            &m_dBgraph, SLOT(useLogFreq(bool)));

    connect(minFreq, SIGNAL(valueChanged(double)),
            &m_phaseGraph, SLOT(setXMin(double)));
    connect(maxFreq, SIGNAL(valueChanged(double)),
            &m_phaseGraph, SLOT(setXMax(double)));
    connect(phaseMin, SIGNAL(valueChanged(double)),
            &m_phaseGraph, SLOT(setYMin(double)));
    connect(phaseMax, SIGNAL(valueChanged(double)),
            &m_phaseGraph, SLOT(setYMax(double)));
    connect(ui->btnIsLog, SIGNAL(toggled(bool)),
            &m_phaseGraph, SLOT(useLogFreq(bool)));

    connect(ui->cbLineThickness,SIGNAL(currentIndexChanged(int)),&m_dBgraph,SLOT(setThickness(int)));
    connect(ui->cbLineThickness,SIGNAL(currentIndexChanged(int)),&m_phaseGraph,SLOT(setThickness(int)));
    connect(ui->cbLineThickness,SIGNAL(currentIndexChanged(int)),ui->nicholsgraph,SLOT(setThickness(int)));
    connect(ui->cbLineThickness,SIGNAL(currentIndexChanged(int)),ui->xygraph,SLOT(setThickness(int)));

    d_bottomHandlesArea = new HorizHandlesArea(this);
    d_bottomHandlesArea->setMinimumHeight(50);

    ui->gridLayout_plots->addWidget(&m_dBgraph,0,0,1,1);
    ui->gridLayout_plots->addWidget(&m_phaseGraph,1,0,1,1);
    ui->gridLayout_plots->addWidget(d_bottomHandlesArea,2,0,1,1);

    d_hCursorHandle1 = new PlotLineHandleH(
            QPixmap(":/icons/h_cursor_handle.svg"),
            d_bottomHandlesArea);
    d_hCursorHandle2 = new PlotLineHandleH(
            QPixmap(":/icons/h_cursor_handle.svg"),
            d_bottomHandlesArea);

    QPen cursorsLinePen = QPen(QColor(155,155,155),1,Qt::DashLine);
    d_hCursorHandle1->setPen(cursorsLinePen);
    d_hCursorHandle2->setPen(cursorsLinePen);
    d_hCursorHandle1->setVisible(false);
    d_hCursorHandle2->setVisible(false);

    connect(&m_dBgraph,SIGNAL(VBar1PixelPosChanged(int)),
            SLOT(onVbar1PixelPosChanged(int)));
    connect(&m_dBgraph,SIGNAL(VBar2PixelPosChanged(int)),
            SLOT(onVbar2PixelPosChanged(int)));

    connect(d_hCursorHandle1, SIGNAL(positionChanged(int)),&m_dBgraph, SLOT(onCursor1PositionChanged(int)));
    connect(d_hCursorHandle2, SIGNAL(positionChanged(int)),&m_dBgraph, SLOT(onCursor2PositionChanged(int)));
    connect(d_hCursorHandle1, SIGNAL(positionChanged(int)),&m_phaseGraph, SLOT(onCursor1PositionChanged(int)));
    connect(d_hCursorHandle2, SIGNAL(positionChanged(int)),&m_phaseGraph, SLOT(onCursor2PositionChanged(int)));

        maxFreq->setMaxValue((double) max_samplerate / 2.5 - 1.0);

	connect(minFreq, SIGNAL(valueChanged(double)),
			this, SLOT(updateNumSamples()));
	connect(maxFreq, SIGNAL(valueChanged(double)),
			this, SLOT(updateNumSamples()));
	connect(samplesCount, SIGNAL(valueChanged(double)),
			this, SLOT(updateNumSamples()));
    connect(ui->boxCursors,SIGNAL(toggled(bool)),
            SLOT(toggleCursors(bool)));

	connect(ui->cmb_graphs,SIGNAL(currentIndexChanged(int)),SLOT(onGraphIndexChanged(int)));

    readPreferences();

	api->setObjectName(QString::fromStdString(Filter::tool_name(
			TOOL_NETWORK_ANALYZER)));

	ui->xygraph->enableZooming(ui->btnZoomIn, ui->btnZoomOut);

	api->load(*settings);
	api->js_register(engine);

    connect((m_dBgraph.getAxisWidget(QwtPlot::xTop)) , SIGNAL(scaleDivChanged () ), &m_phaseGraph, SLOT(scaleDivChanged() ));
    connect((m_phaseGraph.getAxisWidget(QwtPlot::xTop)) , SIGNAL(scaleDivChanged () ), &m_dBgraph, SLOT(scaleDivChanged() ));

    connect(&m_dBgraph,SIGNAL(resetZoom()),&m_phaseGraph,SLOT(onResetZoom()));
    connect(&m_phaseGraph,SIGNAL(resetZoom()),&m_dBgraph,SLOT(onResetZoom()));


    connect(ui->rightMenu, &MenuAnim::finished, this, &NetworkAnalyzer::rightMenuFinished);

    connect(ui->btnSettings, &CustomPushButton::toggled, [=](bool checked){
            triggerRightMenuToggle(ui->btnSettings, checked);
    });
    connect(ui->btnGeneralSettings, &CustomPushButton::toggled, [=](bool checked){
            triggerRightMenuToggle(ui->btnGeneralSettings, checked);
    });
    connect(ui->btnCursors, &CustomPushButton::toggled, [=](bool checked){
            triggerRightMenuToggle(ui->btnCursors, checked);
    });

    ui->btnSettings->setProperty("id",QVariant(-1));
    ui->btnGeneralSettings->setProperty("id",QVariant(-2));
    ui->btnCursors->setProperty("id",QVariant(-3));


    connect(ui->horizontalSlider, &QSlider::valueChanged, [=](int value){
        ui->transLabel->setText("Transparency " + QString::number(value) + "%");
        m_dBgraph.setCursorReadoutsTransparency(value);
        m_phaseGraph.setCursorReadoutsTransparency(value);
    });

    connect(ui->posSelect, &CustomPlotPositionButton::positionChanged,
        [=](CustomPlotPositionButton::ReadoutsPosition position){
        m_dBgraph.moveCursorReadouts(position);
        m_phaseGraph.moveCursorReadouts(position);
    });

    if (!wheelEventGuard)
            wheelEventGuard = new MouseWheelWidgetGuard(ui->mainWidget);
    wheelEventGuard->installEventRecursively(ui->mainWidget);

    connect(ui->btnPrint, &QPushButton::clicked, [=]() {
        QWidget *widget = ui->stackedWidget->currentWidget();
        QImage img (widget->width(), widget->height(), QImage::Format_ARGB32);
        QPainter painter(&img);
        img.fill(Qt::black);
        widget->render(&painter);
        QString date = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");

        QString fileNameHint = "Scopy-" + api->objectName() + "-" + date + ".png";

	QString fileName = QFileDialog::getSaveFileName(this,
				tr("Save to"), fileNameHint,
				tr({"(*.png);;"}));
	painter.end();
	img.invertPixels(QImage::InvertRgb);
	img.save(fileName, 0, -1);
    });


//	top_block = gr::make_top_block("Signal Generator");

}

NetworkAnalyzer::~NetworkAnalyzer()
{
    disconnect(prefPanel,&Preferences::notify,this,&NetworkAnalyzer::readPreferences);
	ui->run_button->setChecked(false);
	if (saveOnExit) {
		api->save(*settings);
	}
	delete api;

	delete ui;
}

void NetworkAnalyzer::setMinimumDistanceBetween(SpinBoxA *min, SpinBoxA *max, double distance)
{

	connect(max, &SpinBoxA::valueChanged, [=](double value) {
	    min->setMaxValue(value - distance);
	    min->setValue(min->value());
	});
	connect(min, &SpinBoxA::valueChanged, [=](double value) {
	    max->setMinValue(value + distance);
	    max->setValue(max->value());
	});
}

void NetworkAnalyzer::triggerRightMenuToggle(CustomPushButton *btn, bool checked)
{
	if (ui->rightMenu->animInProgress()) {
		menuButtonActions.enqueue(
			QPair<CustomPushButton *, bool>(btn, checked));
	} else {
		toggleRightMenu(btn, checked);
	}
}

void NetworkAnalyzer::toggleRightMenu(CustomPushButton *btn, bool checked)
{
    int id = btn->property("id").toInt();

    if(checked)
        ui->stackedWidget_2->setCurrentIndex(-id-1);

    ui->rightMenu->toggleMenu(checked);
}

void NetworkAnalyzer::rightMenuFinished(bool opened)
{
	Q_UNUSED(opened)

	while(menuButtonActions.size()) {
		auto pair = menuButtonActions.dequeue();
		toggleRightMenu(pair.first, pair.second);
	}
}

void NetworkAnalyzer::showEvent(QShowEvent *event)
{
        d_bottomHandlesArea->setLeftPadding(m_dBgraph.axisWidget(QwtAxisId(QwtPlot::yLeft, 0))->width()
                                + ui->gridLayout_plots->margin()
                                + ui->widgetPlotContainer->layout()->margin() + 1);
        int rightPadding = 0;
        rightPadding = rightPadding + m_dBgraph.width()
                        - m_dBgraph.axisWidget(QwtPlot::yLeft)->width() - m_dBgraph.canvas()->width()
                        - ui->widgetPlotContainer->layout()->margin() ;
        d_bottomHandlesArea->setRightPadding(rightPadding);
        d_hCursorHandle1->setPosition(d_hCursorHandle1->pos().x());
        d_hCursorHandle2->setPosition(d_hCursorHandle2->pos().x());
        Tool::showEvent(event);
}

void NetworkAnalyzer::on_btnExport_clicked()
{
	auto export_dialog( new QFileDialog( this ) );
	export_dialog->setWindowModality( Qt::WindowModal );
	export_dialog->setFileMode( QFileDialog::AnyFile );
	export_dialog->setAcceptMode( QFileDialog::AcceptSave );
	export_dialog->setNameFilters({"Comma-separated values files (*.csv)",
					       "Tab-delimited values files (*.txt)"});

	if (export_dialog->exec()) {
		FileManager fm("Network Analyzer");

		fm.open(export_dialog->selectedFiles().at(0), FileManager::EXPORT);

		fm.setAdditionalInformation(ui->btnRefChn->isChecked() ? "Reference channel: 1" : "Reference channel: 2");

		fm.save(m_dBgraph.getXAxisData(), "Frequency(Hz)");
		fm.save(m_dBgraph.getYAxisData(), "Magnitude(dB)");
		fm.save(m_phaseGraph.getYAxisData(), "Phase(°)");

		fm.performWrite();
	}
}

void NetworkAnalyzer::printFrequencyArray()
{

	getFrequencyArray();

}

void NetworkAnalyzer::getFrequencyArray()
{
	QVector<double> ret;
	iterations.clear();

	unsigned int steps = (unsigned int) samplesCount->value();
	double min_freq = minFreq->value();
	double max_freq = maxFreq->value();
	double log10_min_freq = log10(min_freq);
	double log10_max_freq = log10(max_freq);
	double step;

	bool is_log = ui->btnIsLog->isChecked();
	if (is_log)
		step = (log10_max_freq - log10_min_freq) / (double)(steps - 1);
	else
		step = (max_freq - min_freq) / (double)(steps - 1);

	for (unsigned int i = 0; i < steps; ++i) {
		double frequency;
		if (is_log) {
			frequency = pow(10.0,
					log10_min_freq + (double) i * step);
		} else {
			frequency = min_freq + (double) i * step;
		}
		ret.push_back(frequency);
	}



	QVector<double> adjFreq;
	adjFreq.resize(ret.size());

	auto sampleRates = SignalGenerator::get_available_sample_rates(iio_channel_get_device(dac1));
	qSort(sampleRates.begin(), sampleRates.end(), qLess<unsigned int>());

	uint32_t lastRate = sampleRates[0];
	for (int i = 0; i < ret.size(); ++i) {
		double freq = ret[i];

		double next = (i != ret.size() - 1)? ret[i+1] : sampleRates.back();
		double prev = (i > 0) ? ret[i - 1] : ret[i];

		next -= ((next - freq) * 0.5); // 50%
		prev += ((freq - prev) * 0.5);

//		std::cout << " prev: " << prev << " current: " << freq << " next: " << next << std::endl;

		bool stop = false;
		adjFreq[i] = ret[i];

		for (int j = 1; j < 1024 && !stop; ++j) {
			double integral;
			double dummy;
			double fract = modf(1.0/(double)j,&dummy);

			for (auto rate : sampleRates) {
				if (rate < lastRate) {
					continue;
				}
				modf(rate / freq, &integral);
				if (integral < 2.5) {
					continue;
				}
				if (integral < 14 && lastRate < sampleRates.back()) {
					continue;
				}

				double newFrequency = rate / (integral + fract);
				if (newFrequency > prev && newFrequency < next && (integral * j > 2)) {
					stop = true;
					adjFreq[i] = newFrequency;
					std::cout << i << " rate : " << rate << "  new freq: " << newFrequency << " old freq: " << freq <<
						     " integral " << integral << " fract " << fract << "(j= " << j << " )" << std::endl;
					size_t bufferSize = integral * j;

					while (bufferSize & 0x3) {
						bufferSize <<= 1;
					}
					while (bufferSize < 1280) {
						bufferSize <<= 1;
					}
					std::cout << "FREQUENCY: " << newFrequency << " BUFFERSIZE: " << bufferSize << " RATE: " << rate << std::endl;

					iterations.push_back(networkIteration(newFrequency, rate, bufferSize));
					lastRate = rate;
					break;
				}
			}

		}
		if (!stop) {
//			std::cout << "ERROR for frq " << freq << std::endl;
		}
	}
}

size_t NetworkAnalyzer::get_samples_count(double frequency, const struct iio_device *dev, double best_rate, bool perfect)
{
	size_t size = 1;
	size_t max_buffer_size = 128 * 1024 /
			(size_t) iio_device_get_sample_size(dev);


	double ratio, fract;
	ratio = best_rate / frequency;
	ratio = SignalGenerator::get_best_ratio(ratio, (double)(max_buffer_size / 4), &fract);

	if (perfect && fract != 0.0) {
		return 0;
	}

	size = (size_t)ratio;

	while (size & 0x3) {
		size <<= 1;
	}

	/* The buffer size shouldn't be too small */
	while (size < 1024) {
		size <<= 1;
	}

	if (size > max_buffer_size) {
		return 0;
	}

	return size;
}

double NetworkAnalyzer::get_best_rate(const struct iio_device *dev, double frequency)
{
	QVector<unsigned long> values = SignalGenerator::get_available_sample_rates(dev);

	for (unsigned long rate : values) {
		size_t buf_size = get_samples_count(frequency, dev, rate, true);

		if (buf_size) {
			return rate;
		}

		qDebug(CAT_SIGNAL_GENERATOR) << QString("Rate %1 not ideal").arg(rate);
	}

	qSort(values.begin(), values.end(), qGreater<unsigned long>());

	for (unsigned long rate : values) {
		size_t buf_size = get_samples_count(frequency, dev, rate);

		if (buf_size) {
			//std::cout << "Using highest rate for frequency " << frequency << " with buffer size: " << buf_size << std::endl;
			return rate;
		}

		qDebug(CAT_SIGNAL_GENERATOR) << QString("Rate %1 not possible").arg(rate);
	}

	throw std::runtime_error("Unable to calculate best sample rate");
}


void NetworkAnalyzer::updateNumSamples()
{
	if (!ui->run_button->isChecked())
		return;

	unsigned int num_samples = (unsigned int) samplesCount->value();

	m_dBgraph.setNumSamples(num_samples);
	m_phaseGraph.setNumSamples(num_samples);
	ui->xygraph->setNumSamples(num_samples);
	ui->nicholsgraph->setNumSamples(num_samples);
}

size_t NetworkAnalyzer::gcd(size_t a, size_t b)
{
	for (;;) {
		if (!a) {
			return b;
		}

		b %= a;

		if (!b) {
			return a;
		}

		a %= b;
	}
}

void NetworkAnalyzer::run()
{
	const struct iio_device *dev1 = iio_channel_get_device(dac1);
	for (unsigned int i = 0; i < iio_device_get_channels_count(dev1); i++) {
		struct iio_channel *each = iio_device_get_channel(dev1, i);

		if (each == dac1 || each == dac2)
			iio_channel_enable(each);
		else
			iio_channel_disable(each);
	}

	const struct iio_device *dev2 = iio_channel_get_device(dac2);
	for (unsigned int i = 0; i < iio_device_get_channels_count(dev2); i++) {
		struct iio_channel *each = iio_device_get_channel(dev2, i);

		if (each == dac1 || each == dac2)
			iio_channel_enable(each);
		else
			iio_channel_disable(each);
	}

	// Adjust the gain of the ADC channels based on sweep settings
	auto m2k_adc = std::dynamic_pointer_cast<M2kAdc>(adc_dev);
	if (m2k_adc) {
		double sweep_ampl = amplitude->value();
		QPair<double, double> range = m2k_adc->inputRange(
				M2kAdc::HIGH_GAIN_MODE);
		double threshold = range.second - range.first;
		M2kAdc::GainMode gain_mode;
		if (sweep_ampl > threshold) {
			gain_mode = M2kAdc::LOW_GAIN_MODE;
		} else {
			gain_mode = M2kAdc::HIGH_GAIN_MODE;
		}
		for (int chn = 0; chn < m2k_adc->numAdcChannels(); chn++) {
			m2k_adc->setChnHwGainMode(chn, gain_mode);
		}
	}

	QVector<unsigned long> sampleRates = SignalGenerator::get_available_sample_rates(adc);
	qSort(sampleRates.begin(), sampleRates.end(), qLess<unsigned long>());

	getFrequencyArray();
	uint32_t fixedRate = sampleRates[0];
	for (unsigned int i = 0; !stop && i < iterations.size(); ++i) {


		unsigned long rate = iterations[i].rate;
		size_t samples_count = iterations[i].bufferSize;
		double frequency = iterations[i].frequency;

		double amplitudeValue = amplitude->value();
		double offsetValue = offset->value();

		if (dev1 != dev2)
			iio_device_attr_write_bool(dev1, "dma_sync", true);

		struct iio_buffer *buf_dac1 = generateSinWave(dev1,
				frequency, amplitudeValue, offsetValue,
				rate, samples_count);
		if (!buf_dac1) {
			qCritical() << "Unable to create DAC buffer";
			break;
		}

		struct iio_buffer *buf_dac2 = nullptr;

		if (dev1 != dev2) {
			buf_dac2 = generateSinWave(dev2, frequency, amplitudeValue,
					offsetValue, rate, samples_count);
			if (!buf_dac2) {
				qCritical() << "Unable to create DAC buffer";
				break;
			}

			iio_device_attr_write_bool(dev1, "dma_sync", false);
		}

		// Compute buffer_size for exactly 2 periods
		// knowing the frequency compute the time it will take to capture 2 periods

		size_t nrOfPeriods = 2; // prefer
		double timePerPeriod = 1.0 / frequency; // TODO: how many decimals?? 2, 3, 4??

		size_t adc_rate = fixedRate;
		size_t minBufferToAcceptRate = 2; // TODO: 2? 3?? 4??
		size_t minBufferSize = 1024;

		uint32_t buffer_size = 0;


//		std::cout << "####COMPUTING BUFF SIZE FOR FREQUENCY: " << frequency << std::endl;

		for (const auto &rate : sampleRates) {
			buffer_size = rate * timePerPeriod * nrOfPeriods;

			if (rate < fixedRate) {
				continue;
			}

			if (rate / frequency < 2.5) {
				continue;
			}

			// to do change 14
			if (rate / frequency < 14 && fixedRate < sampleRates.back()) {
				continue;
			}

			if (buffer_size & 0x3) {
				buffer_size <<= 1;
			}

			while (buffer_size < 1024) {
				buffer_size <<= 1;
			}
			adc_rate = rate;
			if (fixedRate < rate) {
				//std::cout << "@@@@@@@@SAMPLE RATE CHANGED FOR: " << frequency << std::endl;
			}
			fixedRate = rate;
			break;
//			if (buffer_size > 16536) {
//				adc_rate = rate;

//				break;
//			}
		}

//		std::cout << "####COMPUTED BUFF SIZE FOR FREQUENCY: " << frequency << std::endl;
//		std::cout << buffer_size << " Rate: " << adc_rate << std::endl;

		uint32_t maxSR = 100e6;
		adc_dev->setSampleRate(maxSR);
		iio_device_attr_write_double(adc, "oversampling_ratio", maxSR/adc_rate);


		std::cout << "Capturing at frequency: " << frequency << " with buffer size: " << buffer_size
			  << " should take: " << (double)buffer_size / ((double)maxSR / ((double)maxSR/adc_rate))
			  << " s" << std::endl;

		/* Lock the flowgraph if we are already started */
		bool started = iio->started();
		if (started)
			iio->lock();

		if(buffer_size == 0) {
			qDebug(CAT_NETWORK_ANALYZER) << "buffer size 0";
			return;
		}

		auto f2c1 = blocks::float_to_complex::make();
		auto f2c2 = blocks::float_to_complex::make();
		auto id1 = iio->connect(f2c1, 0, 0, true,
				buffer_size);
		auto id2 = iio->connect(f2c2, 1, 0, true,
				buffer_size);

		auto null = blocks::null_source::make(sizeof(float));
		iio->connect(null, 0, f2c1, 1);
		iio->connect(null, 0, f2c2, 1);

		auto cosine = analog::sig_source_c::make(
				(unsigned int) adc_rate,
				gr::analog::GR_COS_WAVE, -frequency, 1.0);

		auto mult1 = blocks::multiply_cc::make();
		iio->connect(f2c1, 0, mult1, 0);
		iio->connect(cosine, 0, mult1, 1);

		auto mult2 = blocks::multiply_cc::make();
		iio->connect(f2c2, 0, mult2, 0);
		iio->connect(cosine, 0, mult2, 1);

		auto signal = boost::make_shared<signal_sample>();
		auto conj = blocks::multiply_conjugate_cc::make();

		auto avg1 = blocks::moving_average_cc::make(buffer_size,
				2.0 / buffer_size, buffer_size);
		auto skiphead3 = blocks::skiphead::make(sizeof(gr_complex),
				buffer_size - 1);
		auto c2m1 = blocks::complex_to_mag_squared::make();

		iio->connect(mult1, 0, avg1, 0);
		iio->connect(avg1, 0, skiphead3, 0);
		iio->connect(skiphead3, 0, c2m1, 0);
		iio->connect(skiphead3, 0, conj, 0);
		iio->connect(c2m1, 0, signal, 0);

		auto avg2 = blocks::moving_average_cc::make(buffer_size,
				2.0 / buffer_size, buffer_size);
		auto skiphead4 = blocks::skiphead::make(sizeof(gr_complex),
				buffer_size - 1);
		auto c2m2 = blocks::complex_to_mag_squared::make();

		iio->connect(mult2, 0, avg2, 0);
		iio->connect(avg2, 0, skiphead4, 0);
		iio->connect(skiphead4, 0, c2m2, 0);
		iio->connect(skiphead4, 0, conj, 1);
		iio->connect(c2m2, 0, signal, 1);

		auto c2a = blocks::complex_to_arg::make();
		iio->connect(conj, 0, c2a, 0);
		iio->connect(c2a, 0, signal, 2);

		bool got_it = false;
		float mag1 = 0.0f, mag2 = 0.0f, phase = 0.0f;

		connect(&*signal, &signal_sample::triggered,
				[&](const std::vector<float> values) {
			mag1 = values[0];
			mag2 = values[1];
			phase = values[2];
			got_it = true;
		});

		QElapsedTimer t;
		t.start();

		iio->start(id1);
		iio->start(id2);


		if (started)
			iio->unlock();

		do {
			QCoreApplication::processEvents();
			QThread::msleep(10);

			if (!ui->run_button->isChecked())
				break;
		} while (!got_it);

		std::cout << "Actual duration: " << t.elapsed() / 1000.0 << " s" << std::endl;

		iio->stop(id1);
		iio->stop(id2);

		started = iio->started();
		if (started)
			iio->lock();
		iio->disconnect(id1);
		iio->disconnect(id2);
		if (started)
			iio->unlock();

		iio_buffer_destroy(buf_dac1);
		if (buf_dac2)
			iio_buffer_destroy(buf_dac2);

		if (!got_it) /* Process was cancelled */
			return;

		double mag;
        if (ui->btnRefChn->isChecked()) {
			phase = -phase;
			mag = 10.0 * log10(mag2) - 10.0 * log10(mag1);
		} else {
			mag = 10.0 * log10(mag1) - 10.0 * log10(mag2);
		}



		double phase_deg = phase * 180.0 / M_PI;

		qDebug(CAT_NETWORK_ANALYZER) << "Frequency" << frequency << "Hz," <<
			adc_rate << "SPS," << buffer_size << "samples," <<
			mag << "Mag," << phase_deg << "Deg";

        QMetaObject::invokeMethod(&m_dBgraph,
				 "plot",
				 Qt::QueuedConnection,
				 Q_ARG(double, frequency),
				 Q_ARG(double, mag));

        QMetaObject::invokeMethod(&m_phaseGraph,
				 "plot",
				 Qt::QueuedConnection,
				 Q_ARG(double, frequency),
				 Q_ARG(double, phase_deg));

		QMetaObject::invokeMethod(ui->xygraph,
				"plot",
				Qt::QueuedConnection,
				Q_ARG(double, phase_deg),
				Q_ARG(double, mag));

		QMetaObject::invokeMethod(ui->nicholsgraph,
				 "plot",
				 Qt::QueuedConnection,
				 Q_ARG(double, phase_deg),
				 Q_ARG(double, mag));

	}

	Q_EMIT sweepDone();
}

void NetworkAnalyzer::startStop(bool pressed)
{
	stop = !pressed;

	if (amp1 && amp2) {
		/* FIXME: TODO: Move this into a HW class / lib M2k */
		iio_channel_attr_write_bool(amp1, "powerdown", !pressed);
		iio_channel_attr_write_bool(amp2, "powerdown", !pressed);
	}

	ui->btnRefChn->setEnabled(!pressed);
	ui->btnIsLog->setEnabled(!pressed);
	maxFreq->setEnabled(!pressed);
	minFreq->setEnabled(!pressed);
	samplesCount->setEnabled(!pressed);
	amplitude->setEnabled(!pressed);
	offset->setEnabled(!pressed);

	if (pressed) {
        m_dBgraph.reset();
        m_phaseGraph.reset();
		ui->xygraph->reset();
		ui->nicholsgraph->reset();
		updateNumSamples();
		configHwForNetworkAnalyzing();
		thd = QtConcurrent::run(this, &NetworkAnalyzer::run);
	} else {
		ui->run_button->setEnabled(false);
		ui->run_button->setText("Stopping");
		QCoreApplication::processEvents();
		thd.waitForFinished();
		ui->run_button->setEnabled(true);
	}

	setDynamicProperty(ui->run_button, "running", pressed);
}

size_t NetworkAnalyzer::get_sin_samples_count(const struct iio_device *dev,
		unsigned long rate, double frequency)
{
	size_t max_buffer_size = 128 * 1024 /
		(size_t) iio_device_get_sample_size(dev);
	double ratio = (double) rate / frequency;
	size_t size;

	if (ratio < 2.5)
		return 0; /* rate too low */

	if (rate <= 10000)
		max_buffer_size = rate / 2; /* 500ms */

	size = (size_t) SignalGenerator::get_best_ratio(ratio,
			(double) (max_buffer_size / 4), nullptr);

	/* The buffer size must be a multiple of 4 */
	while (size & 0x3)
		size <<= 1;

	/* The buffer size shouldn't be too small */
	while (size < SignalGenerator::min_buffer_size)
		size <<= 1;

	if (size > max_buffer_size)
		return 0;

	return size;
}

unsigned long NetworkAnalyzer::get_best_sample_rate(
		const struct iio_device *dev, double frequency)
{
	QVector<unsigned long> values =
		SignalGenerator::get_available_sample_rates(dev);

	/* Return the best sample rate that we can create a buffer for */
	for (unsigned long rate : values) {
		size_t buf_size = get_sin_samples_count(dev, rate, frequency);
		if (buf_size)
			return rate;

		qDebug(CAT_NETWORK_ANALYZER) << QString("Rate %1 too high, trying lower")
			.arg(rate);
	}

	throw std::runtime_error("Unable to calculate best sample rate");
}

struct iio_buffer * NetworkAnalyzer::generateSinWave(
		const struct iio_device *dev, double frequency,
		double amplitude, double offset,
		unsigned long rate, size_t samples_count)
{
	/* Create the IIO buffer */
	struct iio_buffer *buf = iio_device_create_buffer(
			dev, samples_count, true);
	if (!buf)
		return buf;

	std::cout << "started generation of sine wave " << std::endl;

	auto top_block = gr::make_top_block("Signal Generator");

	auto src = analog::sig_source_f::make(rate, analog::GR_SIN_WAVE,
			frequency, amplitude / 2.0, offset);

	iio_device_attr_write_longlong(dev, "oversampling_ratio", 1);

	// DAC_RAW = (-Vout * 2^11) / 5V
	// Multiplying with 16 because the HDL considers the DAC data as 16 bit
	// instead of 12 bit(data is shifted to the left).
	auto f2s = blocks::float_to_short::make(1,
			-1 * (1 << (DAC_BIT_COUNT - 1)) /
			AMPLITUDE_VOLTS * 16 / INTERP_BY_100_CORR);

	auto head = blocks::head::make(
			sizeof(short), samples_count);

	auto vector = blocks::vector_sink_s::make();

	top_block->connect(src, 0, f2s, 0);
	top_block->connect(f2s, 0, head, 0);
	top_block->connect(head, 0, vector, 0);

	top_block->run();

	const std::vector<short> &samples = vector->data();
	const short *data = samples.data();

	// stop
	top_block->stop();
	top_block->wait();
	top_block->disconnect_all();

	for (unsigned int i = 0; i < iio_device_get_channels_count(dev); i++) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);

		if (iio_channel_is_enabled(chn)) {
			iio_channel_write(chn, buf, data,
					samples_count * sizeof(short));
		}
	}

	iio_device_attr_write_longlong(dev, "sampling_frequency", rate);

	iio_buffer_push(buf);

	std::cout << "finished generation and push of sinewave" << std::endl;

	return buf;
}

void NetworkAnalyzer::configHwForNetworkAnalyzing()
{
	auto trigger = adc_dev->getTrigger();
	if (trigger) {
		for (uint i = 0; i < trigger->numChannels(); i++)
			trigger->setTriggerMode(i, HardwareTrigger::ALWAYS);
	}

	auto m2k_adc = std::dynamic_pointer_cast<M2kAdc>(adc_dev);
//	if (m2k_adc) {
//		iio_device_attr_write_longlong(m2k_adc->iio_adc_dev(),
//			"oversampling_ratio", 1);
//	}
}

void NetworkAnalyzer::onVbar1PixelPosChanged(int pos)
{
    d_hCursorHandle1->setPositionSilenty(pos);
}

void NetworkAnalyzer::onVbar2PixelPosChanged(int pos)
{
    d_hCursorHandle2->setPositionSilenty(pos);
}

void NetworkAnalyzer::toggleCursors(bool en)
{
    if(!en){
        ui->btnCursors->setChecked(en);
    }
    if (d_cursorsEnabled != en) {
        d_cursorsEnabled = en;
        m_dBgraph.toggleCursors(en);
        m_phaseGraph.toggleCursors(en);
        d_hCursorHandle1->setVisible(en);
        d_hCursorHandle2->setVisible(en);
        ui->btnCursors->setEnabled(en);
    }

}

void NetworkAnalyzer::readPreferences()
{
    m_dBgraph.setShowZero(prefPanel->getNa_show_zero());
    m_phaseGraph.setShowZero(prefPanel->getNa_show_zero());
}

void NetworkAnalyzer::onGraphIndexChanged(int index){
    ui->stackedWidget->setCurrentIndex(index);
}

