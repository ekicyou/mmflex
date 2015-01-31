using MMF;
using MMF.Controls.Forms;

namespace CGTest
{
    partial class PanelTestForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.splitContainer2 = new System.Windows.Forms.SplitContainer();
            this.leftTop = new D2DSupportedRenderControl();
            this.splitContainer3 = new System.Windows.Forms.SplitContainer();
            this.rightTop = new D2DSupportedRenderControl();
            this.leftBottom = new D2DSupportedRenderControl();
            this.rightBottom = new D2DSupportedRenderControl();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer2)).BeginInit();
            this.splitContainer2.Panel1.SuspendLayout();
            this.splitContainer2.Panel2.SuspendLayout();
            this.splitContainer2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer3)).BeginInit();
            this.splitContainer3.Panel1.SuspendLayout();
            this.splitContainer3.Panel2.SuspendLayout();
            this.splitContainer3.SuspendLayout();
            this.SuspendLayout();
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(0, 0);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.splitContainer2);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.splitContainer3);
            this.splitContainer1.Size = new System.Drawing.Size(467, 434);
            this.splitContainer1.SplitterDistance = 219;
            this.splitContainer1.TabIndex = 0;
            // 
            // splitContainer2
            // 
            this.splitContainer2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer2.Location = new System.Drawing.Point(0, 0);
            this.splitContainer2.Name = "splitContainer2";
            this.splitContainer2.Orientation = System.Windows.Forms.Orientation.Horizontal;
            // 
            // splitContainer2.Panel1
            // 
            this.splitContainer2.Panel1.BackColor = System.Drawing.SystemColors.ActiveCaption;
            this.splitContainer2.Panel1.Controls.Add(this.leftTop);
            // 
            // splitContainer2.Panel2
            // 
            this.splitContainer2.Panel2.BackColor = System.Drawing.SystemColors.ActiveCaption;
            this.splitContainer2.Panel2.Controls.Add(this.leftBottom);
            this.splitContainer2.Size = new System.Drawing.Size(219, 434);
            this.splitContainer2.SplitterDistance = 194;
            this.splitContainer2.TabIndex = 0;
            // 
            // leftTop
            // 
            this.leftTop.BackgroundColor = new SlimDX.Color3(0F, 0F, 0F);
            this.leftTop.Dock = System.Windows.Forms.DockStyle.Fill;
            this.leftTop.DrawSpriteHandler = null;
            this.leftTop.Location = new System.Drawing.Point(0, 0);
            this.leftTop.Name = "leftTop";
            this.leftTop.Size = new System.Drawing.Size(219, 194);
            this.leftTop.TabIndex = 0;
            // 
            // splitContainer3
            // 
            this.splitContainer3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer3.Location = new System.Drawing.Point(0, 0);
            this.splitContainer3.Name = "splitContainer3";
            this.splitContainer3.Orientation = System.Windows.Forms.Orientation.Horizontal;
            // 
            // splitContainer3.Panel1
            // 
            this.splitContainer3.Panel1.BackColor = System.Drawing.SystemColors.ActiveCaption;
            this.splitContainer3.Panel1.Controls.Add(this.rightTop);
            // 
            // splitContainer3.Panel2
            // 
            this.splitContainer3.Panel2.BackColor = System.Drawing.SystemColors.ActiveCaption;
            this.splitContainer3.Panel2.Controls.Add(this.rightBottom);
            this.splitContainer3.Size = new System.Drawing.Size(244, 434);
            this.splitContainer3.SplitterDistance = 192;
            this.splitContainer3.TabIndex = 0;
            // 
            // rightTop
            // 
            this.rightTop.BackgroundColor = new SlimDX.Color3(0F, 0F, 0F);
            this.rightTop.Dock = System.Windows.Forms.DockStyle.Fill;
            this.rightTop.DrawSpriteHandler = null;
            this.rightTop.Location = new System.Drawing.Point(0, 0);
            this.rightTop.Name = "rightTop";
            this.rightTop.Size = new System.Drawing.Size(244, 192);
            this.rightTop.TabIndex = 0;
            // 
            // leftBottom
            // 
            this.leftBottom.BackgroundColor = new SlimDX.Color3(0F, 0F, 0F);
            this.leftBottom.Dock = System.Windows.Forms.DockStyle.Fill;
            this.leftBottom.DrawSpriteHandler = null;
            this.leftBottom.Location = new System.Drawing.Point(0, 0);
            this.leftBottom.Name = "leftBottom";
            this.leftBottom.Size = new System.Drawing.Size(219, 236);
            this.leftBottom.TabIndex = 0;
            // 
            // rightBottom
            // 
            this.rightBottom.BackgroundColor = new SlimDX.Color3(0F, 0F, 0F);
            this.rightBottom.Dock = System.Windows.Forms.DockStyle.Fill;
            this.rightBottom.DrawSpriteHandler = null;
            this.rightBottom.Location = new System.Drawing.Point(0, 0);
            this.rightBottom.Name = "rightBottom";
            this.rightBottom.Size = new System.Drawing.Size(244, 238);
            this.rightBottom.TabIndex = 0;
            // 
            // PanelTestForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(467, 434);
            this.Controls.Add(this.splitContainer1);
            this.Name = "PanelTestForm";
            this.Text = "PanelTestForm";
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.splitContainer2.Panel1.ResumeLayout(false);
            this.splitContainer2.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer2)).EndInit();
            this.splitContainer2.ResumeLayout(false);
            this.splitContainer3.Panel1.ResumeLayout(false);
            this.splitContainer3.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer3)).EndInit();
            this.splitContainer3.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.SplitContainer splitContainer1;
        private System.Windows.Forms.SplitContainer splitContainer2;
        private System.Windows.Forms.SplitContainer splitContainer3;
        private D2DSupportedRenderControl leftTop;
        private D2DSupportedRenderControl rightTop;
        private D2DSupportedRenderControl leftBottom;
        private D2DSupportedRenderControl rightBottom;
    }
}