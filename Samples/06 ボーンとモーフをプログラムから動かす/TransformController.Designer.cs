namespace _06_MoveBoneAndMorphFromCode
{
    partial class TransformController
    {
        /// <summary>
        /// 必要なデザイナー変数です。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 使用中のリソースをすべてクリーンアップします。
        /// </summary>
        /// <param name="disposing">マネージ リソースが破棄される場合 true、破棄されない場合は false です。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows フォーム デザイナーで生成されたコード

        /// <summary>
        /// デザイナー サポートに必要なメソッドです。このメソッドの内容を
        /// コード エディターで変更しないでください。
        /// </summary>
        private void InitializeComponent()
        {
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.bone_x_plus = new System.Windows.Forms.Button();
            this.bone_y_plus = new System.Windows.Forms.Button();
            this.bone_z_plus = new System.Windows.Forms.Button();
            this.flowLayoutPanel2 = new System.Windows.Forms.FlowLayoutPanel();
            this.bone_x_minus = new System.Windows.Forms.Button();
            this.bone_y_minus = new System.Windows.Forms.Button();
            this.bone_z_minus = new System.Windows.Forms.Button();
            this.flowLayoutPanel3 = new System.Windows.Forms.FlowLayoutPanel();
            this.label1 = new System.Windows.Forms.Label();
            this.bone_combo_box = new System.Windows.Forms.ComboBox();
            this.flowLayoutPanel4 = new System.Windows.Forms.FlowLayoutPanel();
            this.label2 = new System.Windows.Forms.Label();
            this.morph_combo_box = new System.Windows.Forms.ComboBox();
            this.morph_track_bar = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel2.SuspendLayout();
            this.flowLayoutPanel3.SuspendLayout();
            this.flowLayoutPanel4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.morph_track_bar)).BeginInit();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel1, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel2, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel3, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel4, 0, 3);
            this.tableLayoutPanel1.Controls.Add(this.morph_track_bar, 0, 4);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 6;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(284, 262);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.bone_x_plus);
            this.flowLayoutPanel1.Controls.Add(this.bone_y_plus);
            this.flowLayoutPanel1.Controls.Add(this.bone_z_plus);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(3, 39);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(278, 43);
            this.flowLayoutPanel1.TabIndex = 1;
            // 
            // bone_x_plus
            // 
            this.bone_x_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_x_plus.ForeColor = System.Drawing.Color.Red;
            this.bone_x_plus.Location = new System.Drawing.Point(3, 3);
            this.bone_x_plus.Name = "bone_x_plus";
            this.bone_x_plus.Size = new System.Drawing.Size(75, 40);
            this.bone_x_plus.TabIndex = 0;
            this.bone_x_plus.Text = "X+";
            this.bone_x_plus.UseVisualStyleBackColor = true;
            this.bone_x_plus.Click += new System.EventHandler(this.bone_x_plus_Click);
            // 
            // bone_y_plus
            // 
            this.bone_y_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_y_plus.ForeColor = System.Drawing.Color.Lime;
            this.bone_y_plus.Location = new System.Drawing.Point(84, 3);
            this.bone_y_plus.Name = "bone_y_plus";
            this.bone_y_plus.Size = new System.Drawing.Size(75, 40);
            this.bone_y_plus.TabIndex = 1;
            this.bone_y_plus.Text = "Y+";
            this.bone_y_plus.UseVisualStyleBackColor = true;
            this.bone_y_plus.Click += new System.EventHandler(this.bone_y_plus_Click);
            // 
            // bone_z_plus
            // 
            this.bone_z_plus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_z_plus.ForeColor = System.Drawing.Color.Blue;
            this.bone_z_plus.Location = new System.Drawing.Point(165, 3);
            this.bone_z_plus.Name = "bone_z_plus";
            this.bone_z_plus.Size = new System.Drawing.Size(75, 40);
            this.bone_z_plus.TabIndex = 2;
            this.bone_z_plus.Text = "Z+";
            this.bone_z_plus.UseVisualStyleBackColor = true;
            this.bone_z_plus.Click += new System.EventHandler(this.bone_z_plus_Click);
            // 
            // flowLayoutPanel2
            // 
            this.flowLayoutPanel2.Controls.Add(this.bone_x_minus);
            this.flowLayoutPanel2.Controls.Add(this.bone_y_minus);
            this.flowLayoutPanel2.Controls.Add(this.bone_z_minus);
            this.flowLayoutPanel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel2.Location = new System.Drawing.Point(3, 88);
            this.flowLayoutPanel2.Name = "flowLayoutPanel2";
            this.flowLayoutPanel2.Size = new System.Drawing.Size(278, 43);
            this.flowLayoutPanel2.TabIndex = 2;
            // 
            // bone_x_minus
            // 
            this.bone_x_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_x_minus.ForeColor = System.Drawing.Color.Red;
            this.bone_x_minus.Location = new System.Drawing.Point(3, 3);
            this.bone_x_minus.Name = "bone_x_minus";
            this.bone_x_minus.Size = new System.Drawing.Size(75, 40);
            this.bone_x_minus.TabIndex = 3;
            this.bone_x_minus.Text = "X-";
            this.bone_x_minus.UseVisualStyleBackColor = true;
            this.bone_x_minus.Click += new System.EventHandler(this.bone_x_minus_Click);
            // 
            // bone_y_minus
            // 
            this.bone_y_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_y_minus.ForeColor = System.Drawing.Color.Lime;
            this.bone_y_minus.Location = new System.Drawing.Point(84, 3);
            this.bone_y_minus.Name = "bone_y_minus";
            this.bone_y_minus.Size = new System.Drawing.Size(75, 40);
            this.bone_y_minus.TabIndex = 4;
            this.bone_y_minus.Text = "Y-";
            this.bone_y_minus.UseVisualStyleBackColor = true;
            this.bone_y_minus.Click += new System.EventHandler(this.bone_y_minus_Click);
            // 
            // bone_z_minus
            // 
            this.bone_z_minus.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.bone_z_minus.ForeColor = System.Drawing.Color.Blue;
            this.bone_z_minus.Location = new System.Drawing.Point(165, 3);
            this.bone_z_minus.Name = "bone_z_minus";
            this.bone_z_minus.Size = new System.Drawing.Size(75, 40);
            this.bone_z_minus.TabIndex = 5;
            this.bone_z_minus.Text = "Z-";
            this.bone_z_minus.UseVisualStyleBackColor = true;
            this.bone_z_minus.Click += new System.EventHandler(this.bone_z_minus_Click);
            // 
            // flowLayoutPanel3
            // 
            this.flowLayoutPanel3.Controls.Add(this.label1);
            this.flowLayoutPanel3.Controls.Add(this.bone_combo_box);
            this.flowLayoutPanel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel3.Location = new System.Drawing.Point(3, 3);
            this.flowLayoutPanel3.Name = "flowLayoutPanel3";
            this.flowLayoutPanel3.Size = new System.Drawing.Size(278, 30);
            this.flowLayoutPanel3.TabIndex = 3;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(3, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(58, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "ボーン変形";
            // 
            // bone_combo_box
            // 
            this.bone_combo_box.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.bone_combo_box.FormattingEnabled = true;
            this.bone_combo_box.Location = new System.Drawing.Point(67, 3);
            this.bone_combo_box.Name = "bone_combo_box";
            this.bone_combo_box.Size = new System.Drawing.Size(121, 20);
            this.bone_combo_box.TabIndex = 1;
            // 
            // flowLayoutPanel4
            // 
            this.flowLayoutPanel4.Controls.Add(this.label2);
            this.flowLayoutPanel4.Controls.Add(this.morph_combo_box);
            this.flowLayoutPanel4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel4.Location = new System.Drawing.Point(3, 137);
            this.flowLayoutPanel4.Name = "flowLayoutPanel4";
            this.flowLayoutPanel4.Size = new System.Drawing.Size(278, 30);
            this.flowLayoutPanel4.TabIndex = 4;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(3, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(56, 12);
            this.label2.TabIndex = 0;
            this.label2.Text = "モーフ変形";
            // 
            // morph_combo_box
            // 
            this.morph_combo_box.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.morph_combo_box.FormattingEnabled = true;
            this.morph_combo_box.Location = new System.Drawing.Point(65, 3);
            this.morph_combo_box.Name = "morph_combo_box";
            this.morph_combo_box.Size = new System.Drawing.Size(121, 20);
            this.morph_combo_box.TabIndex = 1;
            this.morph_combo_box.SelectedValueChanged += new System.EventHandler(this.morph_combo_box_SelectedValueChanged);
            // 
            // morph_track_bar
            // 
            this.morph_track_bar.Dock = System.Windows.Forms.DockStyle.Fill;
            this.morph_track_bar.Location = new System.Drawing.Point(3, 173);
            this.morph_track_bar.Maximum = 1000;
            this.morph_track_bar.Name = "morph_track_bar";
            this.morph_track_bar.Size = new System.Drawing.Size(278, 45);
            this.morph_track_bar.TabIndex = 5;
            this.morph_track_bar.ValueChanged += new System.EventHandler(this.morph_track_bar_ValueChanged);
            // 
            // TransformController
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 262);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "TransformController";
            this.Text = "TransformController";
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel2.ResumeLayout(false);
            this.flowLayoutPanel3.ResumeLayout(false);
            this.flowLayoutPanel3.PerformLayout();
            this.flowLayoutPanel4.ResumeLayout(false);
            this.flowLayoutPanel4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.morph_track_bar)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button bone_x_plus;
        private System.Windows.Forms.Button bone_y_plus;
        private System.Windows.Forms.Button bone_z_plus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel2;
        private System.Windows.Forms.Button bone_x_minus;
        private System.Windows.Forms.Button bone_y_minus;
        private System.Windows.Forms.Button bone_z_minus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel3;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox bone_combo_box;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel4;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ComboBox morph_combo_box;
        private System.Windows.Forms.TrackBar morph_track_bar;
    }
}

