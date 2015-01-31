namespace _05_HowToUpdateCamera
{
    partial class CameraControlSelector
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
            this.basic_camera_controller = new System.Windows.Forms.Button();
            this.vmd_camera_controller = new System.Windows.Forms.Button();
            this.bone_tracker = new System.Windows.Forms.Button();
            this.user_definition = new System.Windows.Forms.Button();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.AutoSize = true;
            this.flowLayoutPanel1.Controls.Add(this.basic_camera_controller);
            this.flowLayoutPanel1.Controls.Add(this.vmd_camera_controller);
            this.flowLayoutPanel1.Controls.Add(this.bone_tracker);
            this.flowLayoutPanel1.Controls.Add(this.user_definition);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(284, 262);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // basic_camera_controller
            // 
            this.basic_camera_controller.Location = new System.Drawing.Point(3, 3);
            this.basic_camera_controller.Name = "basic_camera_controller";
            this.basic_camera_controller.Size = new System.Drawing.Size(150, 30);
            this.basic_camera_controller.TabIndex = 0;
            this.basic_camera_controller.Text = "マウスでカメラを操作";
            this.basic_camera_controller.UseVisualStyleBackColor = true;
            this.basic_camera_controller.Click += new System.EventHandler(this.basic_camera_controller_Click);
            // 
            // vmd_camera_controller
            // 
            this.vmd_camera_controller.Location = new System.Drawing.Point(3, 39);
            this.vmd_camera_controller.Name = "vmd_camera_controller";
            this.vmd_camera_controller.Size = new System.Drawing.Size(150, 30);
            this.vmd_camera_controller.TabIndex = 1;
            this.vmd_camera_controller.Text = "カメラモーションファイル";
            this.vmd_camera_controller.UseVisualStyleBackColor = true;
            this.vmd_camera_controller.Click += new System.EventHandler(this.vmd_camera_controller_Click);
            // 
            // bone_tracker
            // 
            this.bone_tracker.Location = new System.Drawing.Point(3, 75);
            this.bone_tracker.Name = "bone_tracker";
            this.bone_tracker.Size = new System.Drawing.Size(150, 30);
            this.bone_tracker.TabIndex = 2;
            this.bone_tracker.Text = "ボーン追従";
            this.bone_tracker.UseVisualStyleBackColor = true;
            this.bone_tracker.Click += new System.EventHandler(this.bone_tracker_Click);
            // 
            // user_definition
            // 
            this.user_definition.Location = new System.Drawing.Point(3, 111);
            this.user_definition.Name = "user_definition";
            this.user_definition.Size = new System.Drawing.Size(150, 30);
            this.user_definition.TabIndex = 3;
            this.user_definition.Text = "ユーザー定義";
            this.user_definition.UseVisualStyleBackColor = true;
            this.user_definition.Click += new System.EventHandler(this.user_definition_Click);
            // 
            // CameraControlSelector
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 262);
            this.Controls.Add(this.flowLayoutPanel1);
            this.Name = "CameraControlSelector";
            this.Text = "CameraControlSelector";
            this.flowLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button basic_camera_controller;
        private System.Windows.Forms.Button vmd_camera_controller;
        private System.Windows.Forms.Button bone_tracker;
        private System.Windows.Forms.Button user_definition;
    }
}