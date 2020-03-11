function animate(RoBoDyn,t,q,anim_options,varargin)

if anim_options.bool
    
    if nargin == 4
        %fig1 = figure;
        fig1 = figure('units','normalized','outerposition',[0.1 0.1 0.8 0.8]);
    else
        fig1 = varargin{1};
    end
   
   
    if length(t) == 1
        Anim.showmotion_setup(RoBoDyn,t,q,fig1,anim_options); 
        waitforbuttonpress;
    else

        Anim.showmotion_setup(RoBoDyn,t(1),q(:,1),fig1,anim_options)
        waitforbuttonpress;

        if anim_options.save_movie
            v = VideoWriter(anim_options.movie_name,'MPEG-4');
            v.Quality = anim_options.movie_quality;
            v.FrameRate = anim_options.movie_fps;
            open(v);
            frame = getframe(gcf);
            writeVideo(v,frame);
        end


        for i = 1:anim_options.skip_frame:length(t)
            Anim.showmotion_update(RoBoDyn,t(i),q(:,i),anim_options)

            if anim_options.save_movie
                frame = getframe(gcf);
                writeVideo(v,frame);
            end

            pause(0.01)
            %waitforbuttonpress;
        end

        if anim_options.save_movie
            close(v); 
        end


    end

    
end


end




