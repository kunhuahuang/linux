/*
 * STM32 USART
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

#define DRIVER_NAME "stm32_usart"
#define DEV_NAME "ttystm"

/*
 * register offsets
 * RM0090 30.6.8
 */
#define STM32_USART_SR		0x00
#define STM32_USART_DR		0x04
#define STM32_USART_BRR		0x08
#define STM32_USART_CR1		0x0c
#define STM32_USART_CR2		0x10
#define STM32_USART_CR3		0x14
#define STM32_USART_GTPR	0x18

#define STM32_USART_SR_IDLE	(1 << 4)
#define STM32_USART_SR_RXNE	(1 << 5)
#define STM32_USART_SR_TXE	(1 << 7)

#define STM32_USART_CR1_RE	(1 << 2)
#define STM32_USART_CR1_TE	(1 << 3)
#define STM32_USART_CR1_IDLIE	(1 << 4)
#define STM32_USART_CR1_RXNEIE	(1 << 5)
#define STM32_USART_CR1_TXEIE	(1 << 7)
#define STM32_USART_CR1_PS	(1 << 9)
#define STM32_USART_CR1_PCE	(1 << 10)
#define STM32_USART_CR1_M	(1 << 12)
#define STM32_USART_CR1_UE	(1 << 13)

#define STM32_USART_CR2_STOP_2		(2 << 12)
#define STM32_USART_CR2_STOP_MASK	(3 << 12)

#define STM32_USART_CR3_RTSE	(1 << 8)
#define STM32_USART_CR3_CTSE	(1 << 9)

#define to_stm_port(_port) container_of(_port, struct stm32_usart_port, port)

struct stm32_usart_port {
	struct uart_port port;
	struct clk *clk;
};

static struct stm32_usart_port *stm32_usart_ports[8];

static inline struct stm32_usart_port *stm32_usart_port_get(int index)
{
	if (index > 0 && index <= ARRAY_SIZE(stm32_usart_ports))
		return stm32_usart_ports[index - 1];
	else
		return NULL;
}

static inline void stm32_usart_port_set(int index, struct stm32_usart_port *port)
{
	if (index > 0 && index <= ARRAY_SIZE(stm32_usart_ports))
		stm32_usart_ports[index - 1] = port;
}

static inline void stm32_usart_write(struct stm32_usart_port *stm_port,
	u32 value, unsigned offset)
{
	writel_relaxed(value, stm_port->port.membase + offset);
}

static inline u32 stm32_usart_read(struct stm32_usart_port *stm_port,
	unsigned offset)
{
	return readl_relaxed(stm_port->port.membase + offset);
}

static void stm32_usart_putchar(struct stm32_usart_port *stm_port, char ch)
{
	while (!(stm32_usart_read(stm_port, STM32_USART_SR) & STM32_USART_SR_TXE));

	stm32_usart_write(stm_port, ch, STM32_USART_DR);
}

static void stm32_usart_do_rx(struct stm32_usart_port *stm_port)
{
	struct tty_port *tty_port;
	u32 dr;

	//while (!(stm32_usart_read(stm_port, STM32_USART_SR) & STM32_USART_SR_RXNE));

	dr = stm32_usart_read(stm_port, STM32_USART_DR);
	tty_port = &stm_port->port.state->port;
	tty_insert_flip_char(tty_port, dr & 0xff /*XXX*/, TTY_NORMAL);
	tty_flip_buffer_push(tty_port);
}

static void stm32_usart_do_tx(struct stm32_usart_port *stm_port)
{
	struct circ_buf *xmit;
	u32 cr1;

	if (stm_port->port.x_char) {
		stm32_usart_putchar(stm_port, stm_port->port.x_char);
		stm_port->port.x_char = 0;
		stm_port->port.icount.tx++;
		return;
	}

	if (uart_tx_stopped(&stm_port->port)) {
		cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
		cr1 &= ~STM32_USART_CR1_TXEIE;
		stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
		return;
	}

	xmit = &stm_port->port.state->xmit;
	if (!uart_circ_empty(xmit)) {
		stm32_usart_putchar(stm_port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		stm_port->port.icount.tx++;
		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&stm_port->port);
	} else {
		cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
		cr1 &= ~STM32_USART_CR1_TXEIE;
		stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
	}
}

static unsigned int stm32_usart_tx_empty(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 status;

	status = stm32_usart_read(stm_port, STM32_USART_SR);

	return status & STM32_USART_SR_TXE ? TIOCSER_TEMT : 0;
}

static void stm32_usart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int stm32_usart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void stm32_usart_start_tx(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 cr1;

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 |= STM32_USART_CR1_TXEIE;
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);

	stm32_usart_do_tx(stm_port);
}

static void stm32_usart_stop_tx(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 cr1;

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 &= ~STM32_USART_CR1_TXEIE;
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
}

static void stm32_usart_stop_rx(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 cr1;

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 &= ~STM32_USART_CR1_RE;
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
}

static void stm32_usart_break_ctl(struct uart_port *port, int ctl)
{
}

static irqreturn_t stm32_usart_interrupt(int irq, void *data)
{
	struct stm32_usart_port *stm_port = data;
	u32 sr;

	sr = stm32_usart_read(stm_port, STM32_USART_SR);
	if ((sr & STM32_USART_SR_IDLE) || (sr & STM32_USART_SR_RXNE)) {
		stm32_usart_do_rx(stm_port);
	}

	stm32_usart_do_tx(stm_port);

	return IRQ_HANDLED;
}

static int stm32_usart_startup(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 cr1;
	int ret;

	ret = clk_enable(stm_port->clk);
	if (ret) {
		dev_err(stm_port->port.dev, "enabling clock failed\n");
		goto err_clk_enable;
	}

	ret = request_irq(port->irq, stm32_usart_interrupt,
		IRQF_DISABLED | IRQF_IRQPOLL, DRIVER_NAME, port);
	if (ret) {
		dev_err(stm_port->port.dev, "requesting irq failed\n");
		goto err_request_irq;
	}

	stm32_usart_write(stm_port, 0, STM32_USART_SR);
	stm32_usart_write(stm_port, STM32_USART_CR1_TE | STM32_USART_CR1_RE, STM32_USART_CR1);
	stm32_usart_write(stm_port, 0, STM32_USART_CR2);

	stm32_usart_read(stm_port, STM32_USART_SR);
	stm32_usart_read(stm_port, STM32_USART_DR);

	stm32_usart_write(stm_port, 0, STM32_USART_CR3);

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 |= STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE /*| STM32_USART_CR1_RXNEIE*/;
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 |= STM32_USART_CR1_UE;
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);

	return 0;

err_request_irq:
	clk_disable(stm_port->clk);
err_clk_enable:
	return ret;
}

static void stm32_usart_shutdown(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	u32 cr1;

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 &= ~(STM32_USART_CR1_UE | STM32_USART_CR1_RE | STM32_USART_CR1_TE |
		STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE);
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
	stm32_usart_write(stm_port, 0, STM32_USART_SR);

	free_irq(port->irq, port);

	clk_disable(stm_port->clk);
}

static void stm32_usart_set_termios(struct uart_port *port,
	struct ktermios *new, struct ktermios *old)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	unsigned int baud, max_baud;
	u32 cr1, cr2, cr3;

	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	cr1 &= ~(STM32_USART_CR1_M | STM32_USART_CR1_PCE | STM32_USART_CR1_PS);
	if (new->c_cflag & PARENB) {
		cr1 |= STM32_USART_CR1_PCE;
		if (new->c_cflag & PARODD)
			cr1 |= STM32_USART_CR1_PS;
		if ((new->c_cflag & CSIZE) == CS8)
			cr1 |= STM32_USART_CR1_M;
	}
	stm32_usart_write(stm_port, cr1, STM32_USART_CR1);

	cr2 = stm32_usart_read(stm_port, STM32_USART_CR2);
	cr2 &= ~STM32_USART_CR2_STOP_MASK;
	if (new->c_cflag & CSTOPB)
		cr2 |= STM32_USART_CR2_STOP_2;
	stm32_usart_write(stm_port, cr2, STM32_USART_CR2);

	cr3 = stm32_usart_read(stm_port, STM32_USART_CR3);
	cr3 &= ~(STM32_USART_CR3_CTSE | STM32_USART_CR3_RTSE);
	if (new->c_cflag & CRTSCTS)
		cr3 |= (STM32_USART_CR3_CTSE | STM32_USART_CR3_RTSE);
	stm32_usart_write(stm_port, cr3, STM32_USART_CR3);

	max_baud = clk_get_rate(stm_port->clk) >> 4;
	baud = uart_get_baud_rate(port, new, old, 0, max_baud);
	uart_update_timeout(port, new->c_cflag, baud);
	if (tty_termios_baud_rate(new))
		tty_termios_encode_baud_rate(new, baud, baud);
}

static const char *stm32_usart_type(struct uart_port *port)
{
	return port->type == PORT_STM32_USART ? "stm32_usart" : NULL;
}

static int stm32_usart_request_port(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);
	int ret;

	port->membase = ioremap(port->mapbase, 0x1c);
	if (!port->membase) {
		dev_err(port->dev, "failed to remap\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	stm_port->clk = clk_get(port->dev, NULL);
	if (IS_ERR(stm_port->clk)) {
		dev_err(port->dev, "failed to get clock\n");
		ret = PTR_ERR(stm_port->clk);
		goto err_clk_get;
	}

	ret = clk_prepare(stm_port->clk);
	if (ret) {
		dev_err(port->dev, "failed to prepare clock\n");
		goto err_clk_prepare;
	}

	return 0;

err_clk_prepare:
	clk_put(stm_port->clk);
err_clk_get:
	iounmap(port->membase);
err_ioremap:
	return ret;
}

static void stm32_usart_release_port(struct uart_port *port)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);

	clk_unprepare(stm_port->clk);
	clk_put(stm_port->clk);

	iounmap(port->membase);
}

static void stm32_usart_config_port(struct uart_port *port, int type)
{
	if (stm32_usart_request_port(port))
		return;

	if (type & UART_CONFIG_TYPE)
		port->type = PORT_STM32_USART;
}

static int stm32_usart_verify_port(struct uart_port *port,
		struct serial_struct *serinfo)
{
	if (serinfo->type != PORT_UNKNOWN && serinfo->type != PORT_STM32_USART)
		return -EINVAL;

	return 0;
}

static struct uart_ops stm32_usart_ops = {
	.tx_empty = stm32_usart_tx_empty,
	.set_mctrl = stm32_usart_set_mctrl,
	.get_mctrl = stm32_usart_get_mctrl,
	.start_tx = stm32_usart_start_tx,
	.stop_tx = stm32_usart_stop_tx,
	.stop_rx = stm32_usart_stop_rx,
	.break_ctl = stm32_usart_break_ctl,
	.startup = stm32_usart_startup,
	.shutdown = stm32_usart_shutdown,
	.set_termios = stm32_usart_set_termios,
	.type = stm32_usart_type,
	.request_port = stm32_usart_request_port,
	.release_port = stm32_usart_release_port,
	.config_port = stm32_usart_config_port,
	.verify_port = stm32_usart_verify_port,
};

#ifdef CONFIG_SERIAL_STM32_USART_CONSOLE

static void stm32_usart_console_putchar(struct uart_port *port, int ch)
{
	struct stm32_usart_port *stm_port = to_stm_port(port);

	stm32_usart_putchar(stm_port, ch);
}

static void stm32_usart_console_write(struct console *con, const char *s,
	unsigned int count)
{
	struct stm32_usart_port *stm_port;
	u32 cr1, interrupts;

	stm_port = stm32_usart_port_get(con->index);
	cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
	interrupts = cr1 & (STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE);
	if (interrupts) {
		cr1 &= ~interrupts;
		stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
	}

	uart_console_write(&stm_port->port, s, count,
		stm32_usart_console_putchar);

	if (interrupts) {
		cr1 = stm32_usart_read(stm_port, STM32_USART_CR1);
		cr1 |= interrupts;
		stm32_usart_write(stm_port, cr1, STM32_USART_CR1);
	}
}

static int stm32_usart_console_setup(struct console *con, char *options)
{
	struct stm32_usart_port *stm_port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	stm_port = stm32_usart_port_get(con->index);
	if (!stm_port)	{
		pr_warn("No console at %d\n", con->index);
		return -ENODEV;
	}

	ret = clk_prepare(stm_port->clk);
	if (ret) {
		dev_warn(stm_port->port.dev, "failed to prepare clock\n");
		return ret;
	}
	stm_port->port.uartclk = clk_get_rate(stm_port->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&stm_port->port, con, baud, parity, bits, flow);
}

static struct uart_driver stm32_usart_uart_driver;

static struct console stm32_usart_console = {
	.name = DEV_NAME,
	.write = stm32_usart_console_write,
	.device = uart_console_device,
	.setup = stm32_usart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &stm32_usart_uart_driver,
};

#else
#define stm32_usart_console (*(struct console *)NULL)
#endif

static struct uart_driver stm32_usart_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = DEV_NAME,
	.nr = ARRAY_SIZE(stm32_usart_ports),
	.cons = &stm32_usart_console,
};

static int stm32_usart_probe(struct platform_device *pdev)
{
	struct stm32_usart_port *stm_port;
	struct resource *res;
	int irq;
	int ret;

	stm_port = kzalloc(sizeof(*stm_port), GFP_KERNEL);
	if (!stm_port)
		return -ENOMEM;

	stm_port->port.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_warn(&pdev->dev, "failed to determine base address\n");
		ret = -ENODEV;
		goto err_get_mem;
	}
	stm_port->port.mapbase = res->start;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		ret = irq;
		goto err_get_irq;
	}
	stm_port->port.irq = irq;

	stm_port->port.type = PORT_STM32_USART;
	stm_port->port.iotype = UPIO_MEM32;
	stm_port->port.ops = &stm32_usart_ops;
	stm_port->port.flags = UPF_BOOT_AUTOCONF;

	if (pdev->dev.of_node) {
		int id;

		id = of_alias_get_id(pdev->dev.of_node, "serial");
		if (id >= 0) {
			stm_port->port.line = id;
		} else {
			dev_warn(&pdev->dev, "no alias found\n");
		}
	}
	stm32_usart_port_set(stm_port->port.line, stm_port);

	ret = uart_add_one_port(&stm32_usart_uart_driver, &stm_port->port);
	if (ret) {
		stm32_usart_port_set(stm_port->port.line, NULL);
		goto err_add_port;
	}

	platform_set_drvdata(pdev, stm_port);

	return 0;
err_add_port:
err_get_irq:
err_get_mem:
	kfree(stm_port);
	return ret;
}

static int stm32_usart_remove(struct platform_device *pdev)
{
	struct stm32_usart_port *p = platform_get_drvdata(pdev);

	uart_remove_one_port(&stm32_usart_uart_driver, &p->port);

	return 0;
}

static const struct of_device_id stm32_usart_dt_ids[] = {
	{ .compatible = "st,stm32f4-usart" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm32_usart_dt_ids);

static struct platform_driver stm32_usart_plat_driver = {
	.probe = stm32_usart_probe,
	.remove = stm32_usart_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = stm32_usart_dt_ids,
	},
};

static int __init stm32_usart_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&stm32_usart_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&stm32_usart_plat_driver);
	if (ret)
		uart_unregister_driver(&stm32_usart_uart_driver);

	return ret;
}
module_init(stm32_usart_init);

static void __exit stm32_usart_exit(void)
{
	platform_driver_unregister(&stm32_usart_plat_driver);
	uart_unregister_driver(&stm32_usart_uart_driver);
}
module_exit(stm32_usart_exit);

MODULE_LICENSE("GPL");
