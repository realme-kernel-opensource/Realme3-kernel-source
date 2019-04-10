/*
 * MUSB OTG driver debugfs support
 *
 * Copyright 2010 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <xhci.h>

#include <asm/uaccess.h>
#include <linux/usb/hcd.h>
#include <xhci_debug.h>

static struct dentry *xhci_debugfs_root;

#define HOST_CMD_TEST_SE0_NAK    		0x1
#define HOST_CMD_TEST_J          		0x2
#define HOST_CMD_TEST_K          		0x3
#define HOST_CMD_TEST_PACKET     		0x4

#define U3_P1_PMSC_RO_MASK (0x7)
#define U3_P1_PMSC_PORT_TEST_CTRL_OFFSET (28)
#define U3_P1_PMSC_PORT_TEST_MODE_MASK			(0xF << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)
#define U3_P1_PMSC_PORT_TEST_MODE_J_STATE		(1 << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)
#define U3_P1_PMSC_PORT_TEST_MODE_K_STATE		(2 << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)
#define U3_P1_PMSC_PORT_TEST_MODE_SE0_NAK		(3 << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)
#define U3_P1_PMSC_PORT_TEST_MODE_TEST_PACKET	(4 << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)
#define U3_P1_PMSC_PORT_TEST_MODE_FORCE_EN	(5 << U3_P1_PMSC_PORT_TEST_CTRL_OFFSET)


static int f_host_test_mode(struct xhci_hcd *xhci,int case_num)
{
	/* u8 temp_8; */
	u32 cmd;
	u32 temp;
	int port_id;
	u32 __iomem *addr;

	pr_notice("f_host_test_mode ===>\n");
	port_id = 1;

	switch (case_num) {
	case HOST_CMD_TEST_SE0_NAK:
		pr_notice("TEST_SE0_NAK\n");
		/* step 1 : set the Run/Stop in USBCMD to 0 */
		cmd = readl(&xhci->op_regs->command);
		cmd &= ~CMD_RUN;
		cmd = readl(&xhci->op_regs->command);

		/* step 2 : wait for HCHalted */
		xhci_handshake(&xhci->op_regs->status, STS_HALT, 1, XHCI_MAX_HALT_USEC);

		/* msleep(2000) ; */
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_SC */
		temp = readl(addr);

		pr_notice("USB2_PORT_SC(0x%p) is 0x%x\n", addr, temp);

		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_PMSC */
		temp = readl(addr);

		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		/* step 3 : set the corresponding test mode */
		temp = temp & 0x0FFFFFFF;
		temp |= U3_P1_PMSC_PORT_TEST_MODE_SE0_NAK;

		writel(temp, addr);

		temp = readl(addr);
		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);
		break;	
	case HOST_CMD_TEST_J:
		pr_notice("TEST_J\n");
		/* step 1 : set the Run/Stop in USBCMD to 0 */
		cmd = readl(&xhci->op_regs->command);
		cmd &= ~CMD_RUN;
		cmd = readl(&xhci->op_regs->command);

		/* step 2 : wait for HCHalted */
		xhci_handshake(&xhci->op_regs->status, STS_HALT, 1, XHCI_MAX_HALT_USEC);

		/* msleep(2000) ; */
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_SC */
		temp = readl(addr);

		pr_notice("USB2_PORT_SC(0x%p) is 0x%x\n", addr, temp);

		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_PMSC */
		temp = readl(addr);

		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		/* step 3 : set the corresponding test mode */
		temp = temp & 0x0FFFFFFF;
		temp |= U3_P1_PMSC_PORT_TEST_MODE_J_STATE;

		writel(temp, addr);

		temp = readl(addr);
		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		break;
	case HOST_CMD_TEST_K:
		pr_notice("TEST_K\n");
		/* step 1 : set the Run/Stop in USBCMD to 0 */
		cmd = readl(&xhci->op_regs->command);
		cmd &= ~CMD_RUN;
		cmd = readl(&xhci->op_regs->command);

		/* step 2 : wait for HCHalted */
		xhci_handshake(&xhci->op_regs->status, STS_HALT, 1, XHCI_MAX_HALT_USEC);

		/* msleep(2000) ; */
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_SC */
		temp = readl(addr);

		pr_notice("USB2_PORT_SC(0x%p) is 0x%x\n", addr, temp);

		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_PMSC */
		temp = readl(addr);

		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		/* step 3 : set the corresponding test mode */
		temp = temp & 0x0FFFFFFF;
		temp |= U3_P1_PMSC_PORT_TEST_MODE_K_STATE;

		writel(temp, addr);

		temp = readl(addr);
		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);


		break;
	case HOST_CMD_TEST_PACKET:
		pr_notice("TEST_PACKET\n");
		/* step 1 : set the Run/Stop in USBCMD to 0 */
		cmd = readl(&xhci->op_regs->command);
		cmd &= ~CMD_RUN;
		cmd = readl(&xhci->op_regs->command);

		/* step 2 : wait for HCHalted */
		xhci_handshake(&xhci->op_regs->status, STS_HALT, 1, XHCI_MAX_HALT_USEC);

		/* msleep(2000) ; */
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_SC */
		temp = readl(addr);

		pr_notice("USB2_PORT_SC(0x%p) is 0x%x\n", addr, temp);

		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_PMSC */
		temp = readl(addr);

		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		/* step 3 : set the corresponding test mode */
		temp = temp & 0x0FFFFFFF;
		temp |= U3_P1_PMSC_PORT_TEST_MODE_TEST_PACKET;

		writel(temp, addr);

		temp = readl(addr);
		pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

		break;
	default:
		break;

	}

	return 0;
}

static int f_host_test_mode_stop(struct xhci_hcd *xhci)
{
	u32 temp;
	/*u32 halted;*/
	int port_id;
	u32 __iomem *addr;

	pr_notice("f_host_test_mode_stop ===>\n");

	/* temp = readl(U3_P1_PMSC); */
	/* temp = temp & (~U3_P1_PMSC_RO_MASK); */
	/* temp = temp & (~U3_P1_PMSC_PORT_TEST_MODE_MASK); */
	/* writel(temp, U3_P1_PMSC); */

	port_id = 1;

	addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_PMSC */
	temp = readl(addr);

	pr_notice("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

	/* step 3 : set the corresponding test mode */
	temp = temp & 0x0FFFFFFF;

	writel(temp, addr);

	temp = readl(addr);
	pr_err("USB2_PORT_PMSC(0x%p) is 0x%x\n", addr, temp);

	addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * ((port_id - 1) & 0xff);	/* PORT_SC */
	temp = readl(addr);

	pr_notice("USB2_PORT_SC(0x%p) is 0x%x\n", addr, temp);

	writel(temp | (PORT_POWER), addr);	/* turn on port power after leaving test mode */

	return 0;
}


static int musb_test_mode_show(struct seq_file *s, void *unused)
{
	/*struct xhci_hcd *xhci = s->private;*/

	seq_puts(s, "XHCI test mode debug\n");

	return 0;
}


static int xhci_test_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, musb_test_mode_show, inode->i_private);
}

static ssize_t xhci_test_mode_write(struct file *file,
				    const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct xhci_hcd *xhci = s->private;

	char buf[32];
	char cmdbuf[32];
	int td_no;
	
	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	sscanf(buf, "%31s %d", cmdbuf, &td_no);	

	if (!strncmp(cmdbuf, "testmode_start", 14)){
		f_host_test_mode(xhci, td_no);
	}
	
	if (!strncmp(cmdbuf, "testmode_stop", 13)) {
		f_host_test_mode_stop(xhci);
	}

	return count;
}

static const struct file_operations xhci_test_mode_fops = {
	.open = xhci_test_mode_open,
	.write = xhci_test_mode_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int xhci_init_debugfs(struct xhci_hcd *xhci)
{
	struct dentry *root;
	struct dentry *file;
	int ret;

	root = debugfs_create_dir("xhci_dbg", NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}
	

	file = debugfs_create_file("testmode", S_IRUGO | S_IRUSR | S_IWUGO | S_IWUSR, root, xhci, &xhci_test_mode_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err0;
	}

	xhci_debugfs_root = root;

	return 0;

err0:
	return ret;
}

void /* __init_or_exit */ xhci_exit_debugfs(struct xhci_hcd *xhci)
{
	debugfs_remove_recursive(xhci_debugfs_root);
}

#if 0
static int xhci_dbg_probe(struct platform_device *pdev)
{
	int ret = 0;
	/* your code here£¬your should save client in your own way */
	pr_alert("i2c_common device probe\n");
	file = debugfs_create_file("testmode", S_IRUGO | S_IWUSR, root, musb, &musb_test_mode_fops);	
	ret = device_create_file(&pdev->dev, &dev_attr_ut);
	pr_alert("i2c_common device probe ret = %d\n", ret);
	return ret;
}

static int xhci_dbg_remove(struct platform_device *pdev)
{
	int ret = 0;
	/* your code here */
	device_remove_file(&pdev->dev, &dev_attr_ut);
	return ret;
}

static struct platform_driver xhci_dbg_driver = {
	.driver = {
		   .name = "xhci_dbg",
		   .owner = THIS_MODULE,
		   },

	.probe = xhci_dbg_probe,
	.remove = xhci_dbg_remove,
};


/* platform device */
static struct platform_device xhci_dbg_device = {
	.name = "xhci_dbg",
};

static int __init xhci_dbg_init(void)
{
	platform_device_register(&xhci_dbg_device);
	return platform_driver_register(&xhci_dbg_driver);
}

static void __exit xhci_dbg_exit(void)
{
	platform_driver_unregister(&xhci_dbg_driver);
	platform_device_unregister(&xhci_dbg_device);
}
module_init(xhci_dbg_init);
module_exit(xhci_dbg_exit);
#endif
