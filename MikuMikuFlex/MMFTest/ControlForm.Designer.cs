namespace CGTest
{
    partial class ControlForm
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
            this.flowLayoutPanel2 = new System.Windows.Forms.FlowLayoutPanel();
            this.trans_x_minus = new System.Windows.Forms.Button();
            this.trans_y_minus = new System.Windows.Forms.Button();
            this.trans_z_minus = new System.Windows.Forms.Button();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.trans_x_plus = new System.Windows.Forms.Button();
            this.trans_y_plus = new System.Windows.Forms.Button();
            this.trans_z_plus = new System.Windows.Forms.Button();
            this.Model_Load = new System.Windows.Forms.Button();
            this.Motion_Load = new System.Windows.Forms.Button();
            this.frameSelector = new System.Windows.Forms.TrackBar();
            this.frameLabel = new System.Windows.Forms.Label();
            this.play = new System.Windows.Forms.Button();
            this.stop = new System.Windows.Forms.Button();
            this.doTrack = new System.Windows.Forms.Button();
            this.reload_effect = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.light_position = new System.Windows.Forms.Label();
            this.Add2Child = new System.Windows.Forms.Button();
            this.AddAssimpButton = new System.Windows.Forms.Button();
            this.tableLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel2.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.frameSelector)).BeginInit();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel2, 1, 7);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel1, 0, 7);
            this.tableLayoutPanel1.Controls.Add(this.Model_Load, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.Motion_Load, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.frameSelector, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.frameLabel, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.play, 0, 3);
            this.tableLayoutPanel1.Controls.Add(this.stop, 1, 3);
            this.tableLayoutPanel1.Controls.Add(this.doTrack, 1, 4);
            this.tableLayoutPanel1.Controls.Add(this.reload_effect, 0, 5);
            this.tableLayoutPanel1.Controls.Add(this.label1, 0, 6);
            this.tableLayoutPanel1.Controls.Add(this.light_position, 1, 6);
            this.tableLayoutPanel1.Controls.Add(this.Add2Child, 1, 5);
            this.tableLayoutPanel1.Controls.Add(this.AddAssimpButton, 0, 4);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 8;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(648, 495);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // flowLayoutPanel2
            // 
            this.flowLayoutPanel2.Controls.Add(this.trans_x_minus);
            this.flowLayoutPanel2.Controls.Add(this.trans_y_minus);
            this.flowLayoutPanel2.Controls.Add(this.trans_z_minus);
            this.flowLayoutPanel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel2.Location = new System.Drawing.Point(327, 221);
            this.flowLayoutPanel2.Name = "flowLayoutPanel2";
            this.flowLayoutPanel2.Size = new System.Drawing.Size(318, 271);
            this.flowLayoutPanel2.TabIndex = 10;
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
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.trans_x_plus);
            this.flowLayoutPanel1.Controls.Add(this.trans_y_plus);
            this.flowLayoutPanel1.Controls.Add(this.trans_z_plus);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(3, 221);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(318, 271);
            this.flowLayoutPanel1.TabIndex = 9;
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
            // Model_Load
            // 
            this.Model_Load.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Model_Load.Location = new System.Drawing.Point(3, 3);
            this.Model_Load.Name = "Model_Load";
            this.Model_Load.Size = new System.Drawing.Size(318, 35);
            this.Model_Load.TabIndex = 0;
            this.Model_Load.Text = "モデル読み込み";
            this.Model_Load.UseVisualStyleBackColor = true;
            // 
            // Motion_Load
            // 
            this.Motion_Load.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Motion_Load.Enabled = false;
            this.Motion_Load.Location = new System.Drawing.Point(327, 3);
            this.Motion_Load.Name = "Motion_Load";
            this.Motion_Load.Size = new System.Drawing.Size(318, 35);
            this.Motion_Load.TabIndex = 1;
            this.Motion_Load.Text = "モーション読み込み\r\n";
            this.Motion_Load.UseVisualStyleBackColor = true;
            // 
            // frameSelector
            // 
            this.frameSelector.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanel1.SetColumnSpan(this.frameSelector, 2);
            this.frameSelector.Location = new System.Drawing.Point(3, 44);
            this.frameSelector.Name = "frameSelector";
            this.frameSelector.Size = new System.Drawing.Size(642, 45);
            this.frameSelector.TabIndex = 2;
            // 
            // frameLabel
            // 
            this.frameLabel.AutoSize = true;
            this.frameLabel.Location = new System.Drawing.Point(327, 92);
            this.frameLabel.Name = "frameLabel";
            this.frameLabel.Size = new System.Drawing.Size(0, 12);
            this.frameLabel.TabIndex = 3;
            // 
            // play
            // 
            this.play.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.play.Location = new System.Drawing.Point(3, 107);
            this.play.Name = "play";
            this.play.Size = new System.Drawing.Size(318, 23);
            this.play.TabIndex = 4;
            this.play.Text = "再生";
            this.play.UseVisualStyleBackColor = true;
            // 
            // stop
            // 
            this.stop.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.stop.Location = new System.Drawing.Point(327, 107);
            this.stop.Name = "stop";
            this.stop.Size = new System.Drawing.Size(318, 23);
            this.stop.TabIndex = 5;
            this.stop.Text = "停止";
            this.stop.UseVisualStyleBackColor = true;
            // 
            // doTrack
            // 
            this.doTrack.Location = new System.Drawing.Point(327, 136);
            this.doTrack.Name = "doTrack";
            this.doTrack.Size = new System.Drawing.Size(136, 23);
            this.doTrack.TabIndex = 7;
            this.doTrack.Text = "トラッキング開始";
            this.doTrack.UseVisualStyleBackColor = true;
            this.doTrack.Click += new System.EventHandler(this.doTrack_Click);
            // 
            // reload_effect
            // 
            this.reload_effect.Location = new System.Drawing.Point(3, 165);
            this.reload_effect.Name = "reload_effect";
            this.reload_effect.Size = new System.Drawing.Size(136, 23);
            this.reload_effect.TabIndex = 8;
            this.reload_effect.Text = "エフェクト再読み込み";
            this.reload_effect.UseVisualStyleBackColor = true;
            this.reload_effect.Click += new System.EventHandler(this.reload_effect_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("MS UI Gothic", 20F);
            this.label1.Location = new System.Drawing.Point(3, 191);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(70, 27);
            this.label1.TabIndex = 11;
            this.label1.Text = "ライト";
            // 
            // light_position
            // 
            this.light_position.AutoSize = true;
            this.light_position.Font = new System.Drawing.Font("MS UI Gothic", 12F);
            this.light_position.Location = new System.Drawing.Point(327, 191);
            this.light_position.Name = "light_position";
            this.light_position.Size = new System.Drawing.Size(42, 16);
            this.light_position.TabIndex = 12;
            this.light_position.Text = "ライト";
            // 
            // Add2Child
            // 
            this.Add2Child.Location = new System.Drawing.Point(326, 164);
            this.Add2Child.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.Add2Child.Name = "Add2Child";
            this.Add2Child.Size = new System.Drawing.Size(153, 24);
            this.Add2Child.TabIndex = 13;
            this.Add2Child.Text = "Childウィンドウに追加";
            this.Add2Child.UseVisualStyleBackColor = true;
            this.Add2Child.Click += new System.EventHandler(this.Add2Child_Click);
            // 
            // AddAssimpButton
            // 
            this.AddAssimpButton.Location = new System.Drawing.Point(3, 136);
            this.AddAssimpButton.Name = "AddAssimpButton";
            this.AddAssimpButton.Size = new System.Drawing.Size(214, 23);
            this.AddAssimpButton.TabIndex = 14;
            this.AddAssimpButton.Text = "Assimpモデルを追加";
            this.AddAssimpButton.UseVisualStyleBackColor = true;
            this.AddAssimpButton.Click += new System.EventHandler(this.AddAssimpButton_Click);
            // 
            // ControlForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(648, 495);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "ControlForm";
            this.Text = "ControlForm";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.ControlForm_FormClosing);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.flowLayoutPanel2.ResumeLayout(false);
            this.flowLayoutPanel1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.frameSelector)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Button Model_Load;
        private System.Windows.Forms.Button Motion_Load;
        private System.Windows.Forms.TrackBar frameSelector;
        private System.Windows.Forms.Label frameLabel;
        private System.Windows.Forms.Button play;
        private System.Windows.Forms.Button stop;
        private System.Windows.Forms.Button doTrack;
        private System.Windows.Forms.Button reload_effect;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel2;
        private System.Windows.Forms.Button trans_x_minus;
        private System.Windows.Forms.Button trans_y_minus;
        private System.Windows.Forms.Button trans_z_minus;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button trans_x_plus;
        private System.Windows.Forms.Button trans_y_plus;
        private System.Windows.Forms.Button trans_z_plus;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label light_position;
        private System.Windows.Forms.Button Add2Child;
        private System.Windows.Forms.Button AddAssimpButton;
    }
}