namespace _04_TransformModelFormCode
{
    partial class Controller
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
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.label1 = new System.Windows.Forms.Label();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.trans_x_plus = new System.Windows.Forms.Button();
            this.trans_y_plus = new System.Windows.Forms.Button();
            this.trans_z_plus = new System.Windows.Forms.Button();
            this.flowLayoutPanel2 = new System.Windows.Forms.FlowLayoutPanel();
            this.trans_x_minus = new System.Windows.Forms.Button();
            this.trans_y_minus = new System.Windows.Forms.Button();
            this.trans_z_minus = new System.Windows.Forms.Button();
            this.flowLayoutPanel3 = new System.Windows.Forms.FlowLayoutPanel();
            this.rotate_x_plus = new System.Windows.Forms.Button();
            this.rotate_y_plus = new System.Windows.Forms.Button();
            this.rotate_z_plus = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.flowLayoutPanel4 = new System.Windows.Forms.FlowLayoutPanel();
            this.rotate_x_minus = new System.Windows.Forms.Button();
            this.rotate_y_minus = new System.Windows.Forms.Button();
            this.rotate_z_minus = new System.Windows.Forms.Button();
            this.tableLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel2.SuspendLayout();
            this.flowLayoutPanel3.SuspendLayout();
            this.flowLayoutPanel4.SuspendLayout();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel4, 0, 5);
            this.tableLayoutPanel1.Controls.Add(this.label1, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel1, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel2, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel3, 0, 4);
            this.tableLayoutPanel1.Controls.Add(this.label2, 0, 3);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 6;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.Size = new System.Drawing.Size(284, 262);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(3, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(53, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "平行移動";
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.trans_x_plus);
            this.flowLayoutPanel1.Controls.Add(this.trans_y_plus);
            this.flowLayoutPanel1.Controls.Add(this.trans_z_plus);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(3, 15);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(278, 45);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // trans_x_plus
            // 
            this.trans_x_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.trans_x_plus.ForeColor = System.Drawing.Color.Red;
            this.trans_x_plus.Location = new System.Drawing.Point(3, 3);
            this.trans_x_plus.Name = "trans_x_plus";
            this.trans_x_plus.Size = new System.Drawing.Size(75, 40);
            this.trans_x_plus.TabIndex = 0;
            this.trans_x_plus.Text = "X+";
            this.trans_x_plus.UseVisualStyleBackColor = true;
            this.trans_x_plus.Click += new System.EventHandler(this.trans_x_plus_Click);
            // 
            // trans_y_plus
            // 
            this.trans_y_plus.Font = new System.Drawing.Font("MS UI Gothic", 20.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.trans_y_plus.ForeColor = System.Drawing.Color.Lime;
            this.trans_y_plus.Location = new System.Drawing.Point(84, 3);
            this.trans_y_plus.Name = "trans_y_plus";
            this.trans_y_plus.Size = new System.Drawing.Size(75, 40);
            this.trans_y_plus.TabIndex = 1;
            this.trans_y_plus.Text = "Y+";
            this.trans_y_plus.UseVisualStyleBackColor = true;
            this.trans_y_plus.Click += new System.EventHandler(this.trans_y_plus_Click);
            // 
            // trans_z_plus
            // 
            this.trans_z_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.trans_z_plus.ForeColor = System.Drawing.Color.Blue;
            this.trans_z_plus.Location = new System.Drawing.Point(165, 3);
            this.trans_z_plus.Name = "trans_z_plus";
            this.trans_z_plus.Size = new System.Drawing.Size(75, 40);
            this.trans_z_plus.TabIndex = 2;
            this.trans_z_plus.Text = "Z+";
            this.trans_z_plus.UseVisualStyleBackColor = true;
            this.trans_z_plus.Click += new System.EventHandler(this.trans_z_plus_Click);
            // 
            // flowLayoutPanel2
            // 
            this.flowLayoutPanel2.Controls.Add(this.trans_x_minus);
            this.flowLayoutPanel2.Controls.Add(this.trans_y_minus);
            this.flowLayoutPanel2.Controls.Add(this.trans_z_minus);
            this.flowLayoutPanel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel2.Location = new System.Drawing.Point(3, 66);
            this.flowLayoutPanel2.Name = "flowLayoutPanel2";
            this.flowLayoutPanel2.Size = new System.Drawing.Size(278, 45);
            this.flowLayoutPanel2.TabIndex = 1;
            // 
            // trans_x_minus
            // 
            this.trans_x_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.trans_x_minus.ForeColor = System.Drawing.Color.Red;
            this.trans_x_minus.Location = new System.Drawing.Point(3, 3);
            this.trans_x_minus.Name = "trans_x_minus";
            this.trans_x_minus.Size = new System.Drawing.Size(75, 40);
            this.trans_x_minus.TabIndex = 0;
            this.trans_x_minus.Text = "X-";
            this.trans_x_minus.UseVisualStyleBackColor = true;
            this.trans_x_minus.Click += new System.EventHandler(this.trans_x_minus_Click);
            // 
            // trans_y_minus
            // 
            this.trans_y_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.trans_y_minus.ForeColor = System.Drawing.Color.Lime;
            this.trans_y_minus.Location = new System.Drawing.Point(84, 3);
            this.trans_y_minus.Name = "trans_y_minus";
            this.trans_y_minus.Size = new System.Drawing.Size(75, 40);
            this.trans_y_minus.TabIndex = 1;
            this.trans_y_minus.Text = "Y-";
            this.trans_y_minus.UseVisualStyleBackColor = true;
            this.trans_y_minus.Click += new System.EventHandler(this.trans_y_minus_Click);
            // 
            // trans_z_minus
            // 
            this.trans_z_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.trans_z_minus.ForeColor = System.Drawing.Color.Blue;
            this.trans_z_minus.Location = new System.Drawing.Point(165, 3);
            this.trans_z_minus.Name = "trans_z_minus";
            this.trans_z_minus.Size = new System.Drawing.Size(75, 40);
            this.trans_z_minus.TabIndex = 2;
            this.trans_z_minus.Text = "Z-";
            this.trans_z_minus.UseVisualStyleBackColor = true;
            this.trans_z_minus.Click += new System.EventHandler(this.trans_z_minus_Click);
            // 
            // flowLayoutPanel3
            // 
            this.flowLayoutPanel3.Controls.Add(this.rotate_x_plus);
            this.flowLayoutPanel3.Controls.Add(this.rotate_y_plus);
            this.flowLayoutPanel3.Controls.Add(this.rotate_z_plus);
            this.flowLayoutPanel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel3.Location = new System.Drawing.Point(3, 129);
            this.flowLayoutPanel3.Name = "flowLayoutPanel3";
            this.flowLayoutPanel3.Size = new System.Drawing.Size(278, 45);
            this.flowLayoutPanel3.TabIndex = 2;
            // 
            // rotate_x_plus
            // 
            this.rotate_x_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.rotate_x_plus.ForeColor = System.Drawing.Color.Red;
            this.rotate_x_plus.Location = new System.Drawing.Point(3, 3);
            this.rotate_x_plus.Name = "rotate_x_plus";
            this.rotate_x_plus.Size = new System.Drawing.Size(75, 40);
            this.rotate_x_plus.TabIndex = 3;
            this.rotate_x_plus.Text = "X+";
            this.rotate_x_plus.UseVisualStyleBackColor = true;
            this.rotate_x_plus.Click += new System.EventHandler(this.rotate_x_plus_Click);
            // 
            // rotate_y_plus
            // 
            this.rotate_y_plus.Font = new System.Drawing.Font("MS UI Gothic", 20.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.rotate_y_plus.ForeColor = System.Drawing.Color.Lime;
            this.rotate_y_plus.Location = new System.Drawing.Point(84, 3);
            this.rotate_y_plus.Name = "rotate_y_plus";
            this.rotate_y_plus.Size = new System.Drawing.Size(75, 40);
            this.rotate_y_plus.TabIndex = 4;
            this.rotate_y_plus.Text = "Y+";
            this.rotate_y_plus.UseVisualStyleBackColor = true;
            this.rotate_y_plus.Click += new System.EventHandler(this.rotate_y_plus_Click);
            // 
            // rotate_z_plus
            // 
            this.rotate_z_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.rotate_z_plus.ForeColor = System.Drawing.Color.Blue;
            this.rotate_z_plus.Location = new System.Drawing.Point(165, 3);
            this.rotate_z_plus.Name = "rotate_z_plus";
            this.rotate_z_plus.Size = new System.Drawing.Size(75, 40);
            this.rotate_z_plus.TabIndex = 5;
            this.rotate_z_plus.Text = "Z+";
            this.rotate_z_plus.UseVisualStyleBackColor = true;
            this.rotate_z_plus.Click += new System.EventHandler(this.rotate_z_plus_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(3, 114);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(29, 12);
            this.label2.TabIndex = 3;
            this.label2.Text = "回転";
            // 
            // flowLayoutPanel4
            // 
            this.flowLayoutPanel4.Controls.Add(this.rotate_x_minus);
            this.flowLayoutPanel4.Controls.Add(this.rotate_y_minus);
            this.flowLayoutPanel4.Controls.Add(this.rotate_z_minus);
            this.flowLayoutPanel4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel4.Location = new System.Drawing.Point(3, 180);
            this.flowLayoutPanel4.Name = "flowLayoutPanel4";
            this.flowLayoutPanel4.Size = new System.Drawing.Size(278, 79);
            this.flowLayoutPanel4.TabIndex = 4;
            // 
            // rotate_x_minus
            // 
            this.rotate_x_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.rotate_x_minus.ForeColor = System.Drawing.Color.Red;
            this.rotate_x_minus.Location = new System.Drawing.Point(3, 3);
            this.rotate_x_minus.Name = "rotate_x_minus";
            this.rotate_x_minus.Size = new System.Drawing.Size(75, 40);
            this.rotate_x_minus.TabIndex = 0;
            this.rotate_x_minus.Text = "X-";
            this.rotate_x_minus.UseVisualStyleBackColor = true;
            this.rotate_x_minus.Click += new System.EventHandler(this.rotate_x_minus_Click);
            // 
            // rotate_y_minus
            // 
            this.rotate_y_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.rotate_y_minus.ForeColor = System.Drawing.Color.Lime;
            this.rotate_y_minus.Location = new System.Drawing.Point(84, 3);
            this.rotate_y_minus.Name = "rotate_y_minus";
            this.rotate_y_minus.Size = new System.Drawing.Size(75, 40);
            this.rotate_y_minus.TabIndex = 1;
            this.rotate_y_minus.Text = "Y-";
            this.rotate_y_minus.UseVisualStyleBackColor = true;
            this.rotate_y_minus.Click += new System.EventHandler(this.rotate_y_minus_Click);
            // 
            // rotate_z_minus
            // 
            this.rotate_z_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.rotate_z_minus.ForeColor = System.Drawing.Color.Blue;
            this.rotate_z_minus.Location = new System.Drawing.Point(165, 3);
            this.rotate_z_minus.Name = "rotate_z_minus";
            this.rotate_z_minus.Size = new System.Drawing.Size(75, 40);
            this.rotate_z_minus.TabIndex = 2;
            this.rotate_z_minus.Text = "Z-";
            this.rotate_z_minus.UseVisualStyleBackColor = true;
            this.rotate_z_minus.Click += new System.EventHandler(this.rotate_z_minus_Click);
            // 
            // Controller
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 262);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "Controller";
            this.Text = "Controller";
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel2.ResumeLayout(false);
            this.flowLayoutPanel3.ResumeLayout(false);
            this.flowLayoutPanel4.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button trans_x_plus;
        private System.Windows.Forms.Button trans_y_plus;
        private System.Windows.Forms.Button trans_z_plus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel2;
        private System.Windows.Forms.Button trans_x_minus;
        private System.Windows.Forms.Button trans_y_minus;
        private System.Windows.Forms.Button trans_z_minus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel4;
        private System.Windows.Forms.Button rotate_x_minus;
        private System.Windows.Forms.Button rotate_y_minus;
        private System.Windows.Forms.Button rotate_z_minus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel3;
        private System.Windows.Forms.Button rotate_x_plus;
        private System.Windows.Forms.Button rotate_y_plus;
        private System.Windows.Forms.Button rotate_z_plus;
        private System.Windows.Forms.Label label2;
    }
}