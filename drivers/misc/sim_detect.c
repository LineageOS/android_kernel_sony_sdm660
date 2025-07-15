/* misc/sim_detect/sim_detect.c
 *
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * Author: Atsushi Iyogi <atsushi.x.iyogi@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/extcon-provider.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include "sim_detect.h"

#define SIM_DETECT_DEV_NAME "sim_detect"

struct sim_detect_event_data {
	const struct sim_detect_gpio_event *event;
	struct timer_list det_timer;
	struct work_struct det_work;
	unsigned int timer_debounce;
	unsigned int irq;
};

struct sim_detect_drvdata {
	struct device *dev;
	struct pinctrl *key_pinctrl;
	struct extcon_dev *edev;
	struct mutex lock;
	atomic_t detection_in_progress;
	unsigned int n_events;
	unsigned int current_state;
	struct sim_detect_event_data data[0];
};

enum sim_detect_switch_state {
	SWITCH_OFF,
	SWITCH_ON,
};

/* Define extcon cable types */
static const unsigned int sim_extcon_cable[] = {
	EXTCON_MECHANICAL,
	EXTCON_NONE,
};

static int sim_detect_gpio_read(struct sim_detect_drvdata *ddata)
{
	int i;
	int gpio_state = 0;

	for (i = 0; i < ddata->n_events; i++)
		gpio_state +=
			(gpio_get_value_cansleep(ddata->data[i].event->gpio)
			 ^ ddata->data[i].event->active_low ?
			SWITCH_ON : SWITCH_OFF) << i;
	return gpio_state;
}

static void sim_detect_report_switch_event(struct sim_detect_drvdata *ddata)
{
	int new_state = 0;
	bool sim_present;

	mutex_lock(&ddata->lock);

	new_state = sim_detect_gpio_read(ddata);
	dev_dbg(ddata->dev, "%s: current value(%d) new value(%d)\n",
		__func__, ddata->current_state, new_state);

	if (new_state == ddata->current_state)
		goto skip_report;

	sim_present = new_state > ddata->current_state;

	dev_info(ddata->dev, "%s: SIM %s\n", __func__,
		 sim_present ? "inserted" : "removed");

	extcon_set_state_sync(ddata->edev, EXTCON_MECHANICAL, sim_present);
	ddata->current_state = new_state;

skip_report:
	mutex_unlock(&ddata->lock);
}

static int sim_detect_get_devtree(struct device *dev,
				  struct sim_detect_platform_data *pdata)
{
	struct device_node *node, *pp = NULL;
	int i = 0;
	struct sim_detect_gpio_event *events;
	u32 reg;
	int gpio;
	int ret = -ENODEV;
	enum of_gpio_flags flags;

	node = dev->of_node;
	if (node == NULL)
		goto fail;

	memset(pdata, 0, sizeof(*pdata));

	pdata->n_events = 0;
	pp = NULL;
	while ((pp = of_get_next_child(node, pp)))
		pdata->n_events++;

	if (pdata->n_events == 0)
		goto fail;

	events = kzalloc(pdata->n_events * sizeof(*events), GFP_KERNEL);
	if (!events) {
		ret = -ENOMEM;
		goto fail;
	}

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->n_events--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "%s: invalid gpio %d\n", __func__, gpio);
			goto out_fail;
		}
		events[i].index = i;
		events[i].gpio = gpio;
		events[i].active_low = flags & OF_GPIO_ACTIVE_LOW;

		events[i].desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "debounce-interval", &reg) == 0)
			events[i].debounce_interval = reg;
		i++;
	}
	pdata->events = events;

	return 0;

out_fail:
	kfree(events);
fail:
	return ret;
}

static irqreturn_t sim_detect_isr(int irq, void *data)
{
	struct sim_detect_event_data *edata =
			(struct sim_detect_event_data *)data;
	const struct sim_detect_gpio_event *event = edata->event;
	struct sim_detect_drvdata *ddata = container_of(edata,
						struct sim_detect_drvdata,
						data[event->index]);

	if (edata->timer_debounce)
		mod_timer(&edata->det_timer,
			jiffies + msecs_to_jiffies(edata->timer_debounce));
	else
		schedule_work(&edata->det_work);

	atomic_set(&ddata->detection_in_progress, 1);

	return IRQ_HANDLED;
}

static void sim_detect_det_tmr_func(struct timer_list *t)
{
	struct sim_detect_event_data *edata =
		from_timer(edata, t, det_timer);

	schedule_work(&edata->det_work);
}

static void sim_detect_det_work(struct work_struct *work)
{
	struct sim_detect_event_data *edata =
		container_of(work, struct sim_detect_event_data, det_work);
	const struct sim_detect_gpio_event *event = edata->event;
	struct sim_detect_drvdata *ddata = container_of(edata,
						struct sim_detect_drvdata,
						data[event->index]);

	sim_detect_report_switch_event(ddata);

	atomic_set(&ddata->detection_in_progress, 0);
}

static int sim_detect_pinctrl_configure(struct sim_detect_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_sim_detect_active");
		if (IS_ERR(set_state)) {
			dev_err(ddata->dev,
				"cannot get ts pinctrl active state\n");
			goto lookup_err;
		}
	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_sim_detect_suspend");
		if (IS_ERR(set_state)) {
			dev_err(ddata->dev,
				"cannot get gpiokey pinctrl sleep state\n");
			goto lookup_err;
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		dev_err(ddata->dev,
				"cannot set ts pinctrl active state\n");
		goto select_err;
	}

	return 0;

lookup_err:
	return PTR_ERR(set_state);
select_err:
	return retval;
}

static int sim_detect_setup_event(struct platform_device *pdev,
				  struct sim_detect_event_data *edata,
				  const struct sim_detect_gpio_event *event)
{
	const char *desc = event->desc ? event->desc : SIM_DETECT_DEV_NAME;
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	edata->event = event;

	error = devm_gpio_request(dev, event->gpio, desc);
	if (error < 0) {
		dev_warn(dev, "Failed to request GPIO %d, error %d\n",
			event->gpio, error);
		return error;
	}

	error = gpio_direction_input(event->gpio);
	if (error < 0) {
		dev_err(dev,
			"Failed to configure direction for GPIO %d, error %d\n",
			event->gpio, error);
		return error;
	}

	edata->timer_debounce = event->debounce_interval;

	irq = gpio_to_irq(event->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			event->gpio, error);
		return error;
	}
	edata->irq = irq;

	isr = sim_detect_isr;
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
#ifdef CONFIG_TRAY_SHARED_INTERRUPT_DETECT
	irqflags |= IRQF_SHARED;
#endif

	INIT_WORK(&edata->det_work, sim_detect_det_work);

	timer_setup(&edata->det_timer, sim_detect_det_tmr_func, 0);

	error = devm_request_any_context_irq(dev, edata->irq, isr, irqflags,
					     desc, edata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			edata->irq, error);
		return error;
	}
	enable_irq_wake(edata->irq);

	return 0;
}

static void sim_detect_remove_event(struct sim_detect_event_data *edata)
{
	if (edata->timer_debounce)
		del_timer_sync(&edata->det_timer);
	cancel_work_sync(&edata->det_work);
}

static ssize_t sim_state_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct sim_detect_drvdata *ddata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", ddata->current_state);
}

static DEVICE_ATTR_RO(sim_state);

static struct attribute *sim_detect_attrs[] = {
	&dev_attr_sim_state.attr,
	NULL,
};

static struct attribute_group sim_detect_attr_group = {
	.attrs = sim_detect_attrs,
};

static int sim_detect_probe(struct platform_device *pdev)
{
	struct sim_detect_platform_data *pdata = pdev->dev.platform_data;
	struct sim_detect_platform_data alt_pdata;
	const struct sim_detect_gpio_event *event;
	struct sim_detect_event_data *edata;
	struct sim_detect_drvdata *ddata;
	int i = 0;
	int error = 0;
	struct pinctrl_state *set_state;

	if (!pdata) {
		error = sim_detect_get_devtree(&pdev->dev, &alt_pdata);
		if (error)
			return error;
		pdata = &alt_pdata;
	}

	ddata = devm_kzalloc(&pdev->dev, sizeof(struct sim_detect_drvdata) +
			pdata->n_events * sizeof(struct sim_detect_event_data),
			GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev, "failed to allocate drvdata in probe\n");
		error = -ENOMEM;
		goto fail_free_pdata;
	}

	ddata->dev = &pdev->dev;
	ddata->n_events = pdata->n_events;
	ddata->key_pinctrl = devm_pinctrl_get(ddata->dev);
	mutex_init(&ddata->lock);

	platform_set_drvdata(pdev, ddata);

	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER) {
			error = -EPROBE_DEFER;
			goto fail_mutex;
		}
		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = sim_detect_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(ddata->dev,
				"cannot set ts pinctrl active state\n");
			goto fail_mutex;
		}
	}

	/* Allocate extcon device */
	ddata->edev = devm_extcon_dev_allocate(ddata->dev, sim_extcon_cable);
	if (IS_ERR(ddata->edev)) {
		error = PTR_ERR(ddata->edev);
		dev_err(ddata->dev, "failed to allocate extcon device\n");
		goto fail_pinctrl;
	}

	/* Register extcon device */
	error = devm_extcon_dev_register(ddata->dev, ddata->edev);
	if (error) {
		dev_err(ddata->dev, "failed to register extcon device\n");
		goto fail_pinctrl;
	}

	error = sysfs_create_group(&ddata->dev->kobj, &sim_detect_attr_group);
	if (error) {
		dev_err(ddata->dev, "%s: sysfs_create_group failed %d\n",
			__func__, error);
		goto fail_pinctrl;
	}

	for (i = 0; i < pdata->n_events; i++) {
		event = &pdata->events[i];
		edata = &ddata->data[i];

		error = sim_detect_setup_event(pdev, edata, event);
		if (error) {
			dev_err(ddata->dev, "%s cannot set event error(%d)\n",
				__func__, error);
			goto fail_setup_event;
		}
	}
	ddata->current_state = sim_detect_gpio_read(ddata);

	/* Set initial state */
	extcon_set_state_sync(ddata->edev, EXTCON_MECHANICAL,
			      ddata->current_state > 0);

	dev_info(ddata->dev, "sim_detect driver was successful.\n");
	return 0;

fail_setup_event:
	while (--i >= 0)
		sim_detect_remove_event(&ddata->data[i]);
	sysfs_remove_group(&ddata->dev->kobj, &sim_detect_attr_group);
fail_pinctrl:
	if (ddata->key_pinctrl) {
		set_state =
		pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_sim_detect_suspend");
		if (!IS_ERR(set_state))
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}
fail_mutex:
	mutex_destroy(&ddata->lock);
fail_free_pdata:
	if (!pdev->dev.platform_data && pdata)
		kfree(pdata->events);
	return error;
}

static int sim_detect_remove(struct platform_device *pdev)
{
	int i;
	struct sim_detect_drvdata *ddata = platform_get_drvdata(pdev);
	struct sim_detect_platform_data *pdata = pdev->dev.platform_data;

	sysfs_remove_group(&ddata->dev->kobj, &sim_detect_attr_group);

	for (i = 0; i < ddata->n_events; i++)
		sim_detect_remove_event(&ddata->data[i]);

	if (ddata->key_pinctrl) {
		struct pinctrl_state *set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
					     "tlmm_sim_detect_suspend");
		if (!IS_ERR(set_state))
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}

	mutex_destroy(&ddata->lock);

	if (!pdata && ddata->data[0].event)
		kfree(ddata->data[0].event);

	return 0;
}

static int __maybe_unused sim_detect_suspend(struct device *dev)
{
	struct sim_detect_drvdata *ddata = dev_get_drvdata(dev);
	int ret = 0;

	if (atomic_read(&ddata->detection_in_progress)) {
		dev_dbg(dev, "detection in progress. (%s)\n", __func__);
		ret = -EAGAIN;
		goto out;
	}

	if (ddata->key_pinctrl) {
		ret = sim_detect_pinctrl_configure(ddata, false);
		if (ret)
			dev_err(dev, "failed to put the pin\n");
	}

out:
	return ret;
}

static int __maybe_unused sim_detect_resume(struct device *dev)
{
	struct sim_detect_drvdata *ddata = dev_get_drvdata(dev);
	int ret = 0;

	if (ddata->key_pinctrl) {
		ret = sim_detect_pinctrl_configure(ddata, true);
		if (ret)
			dev_err(dev, "failed to put the pin\n");
	}

	return ret;
}

static SIMPLE_DEV_PM_OPS(sim_detect_pm_ops, sim_detect_suspend, sim_detect_resume);

static const struct of_device_id sim_detect_match_table[] = {
	{ .compatible = "sim-detect", },
	{ }
};
MODULE_DEVICE_TABLE(of, sim_detect_match_table);

static struct platform_driver sim_detect_driver = {
	.probe = sim_detect_probe,
	.remove = sim_detect_remove,
	.driver = {
		.name = SIM_DETECT_DEV_NAME,
		.pm = &sim_detect_pm_ops,
		.of_match_table = sim_detect_match_table,
	},
};

module_platform_driver(sim_detect_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SIM card detection driver");
MODULE_AUTHOR("Atsushi Iyogi <atsushi.x.iyogi@sonymobile.com>");
