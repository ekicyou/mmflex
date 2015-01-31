namespace _08_MultiScreenRendering
{
    partial class ControllerForm
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
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.Add2Form1 = new System.Windows.Forms.Button();
            this.Add2ChildForm = new System.Windows.Forms.Button();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.Add2Form1);
            this.flowLayoutPanel1.Controls.Add(this.Add2ChildForm);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(284, 262);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // Add2Form1
            // 
            this.Add2Form1.Location = new System.Drawing.Point(3, 3);
            this.Add2Form1.Name = "Add2Form1";
            this.Add2Form1.Size = new System.Drawing.Size(112, 50);
            this.Add2Form1.TabIndex = 0;
            this.Add2Form1.Text = "Form1に追加";
            this.Add2Form1.UseVisualStyleBackColor = true;
            this.Add2Form1.Click += new System.EventHandler(this.Add2Form1_Click);
            // 
            // Add2ChildForm
            // 
            this.Add2ChildForm.Location = new System.Drawing.Point(121, 3);
            this.Add2ChildForm.Name = "Add2ChildForm";
            this.Add2ChildForm.Size = new System.Drawing.Size(112, 50);
            this.Add2ChildForm.TabIndex = 1;
            this.Add2ChildForm.Text = "ChildFormに追加";
            this.Add2ChildForm.UseVisualStyleBackColor = true;
            this.Add2ChildForm.Click += new System.EventHandler(this.Add2ChildForm_Click);
            // 
            // ControllerForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 262);
            this.Controls.Add(this.flowLayoutPanel1);
            this.Name = "ControllerForm";
            this.Text = "ControllerForm";
            this.flowLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button Add2Form1;
        private System.Windows.Forms.Button Add2ChildForm;
    }
}